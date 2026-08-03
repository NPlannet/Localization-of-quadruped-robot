import json
import math
import os
import statistics
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def circular_mean(angles: Sequence[float]) -> float:
    return math.atan2(
        sum(math.sin(angle) for angle in angles),
        sum(math.cos(angle) for angle in angles),
    )


def circular_std(angles: Sequence[float]) -> float:
    if len(angles) < 2:
        return 0.0
    mean_cos = sum(math.cos(angle) for angle in angles) / len(angles)
    mean_sin = sum(math.sin(angle) for angle in angles) / len(angles)
    resultant = min(max(math.hypot(mean_cos, mean_sin), 1e-12), 1.0)
    return math.sqrt(max(0.0, -2.0 * math.log(resultant)))


def percentile(values: Sequence[float], percentage: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def error_metrics(errors: Sequence[float]) -> Dict[str, Optional[float]]:
    if not errors:
        return {
            'mae_m': None,
            'rmse_m': None,
            'median_m': None,
            'p95_m': None,
            'max_m': None,
            'std_m': None,
        }
    return {
        'mae_m': sum(errors) / len(errors),
        'rmse_m': math.sqrt(sum(error * error for error in errors) / len(errors)),
        'median_m': statistics.median(errors),
        'p95_m': percentile(errors, 95.0),
        'max_m': max(errors),
        'std_m': statistics.pstdev(errors) if len(errors) > 1 else 0.0,
    }


def fit_se2(
    estimated_points: Sequence[Tuple[float, float]],
    ground_truth_points: Sequence[Tuple[float, float]],
) -> Dict[str, float]:
    """Fit the rigid transform that maps estimated 2D points to ground truth."""
    if len(estimated_points) != len(ground_truth_points) or not estimated_points:
        raise ValueError('SE(2) alignment needs equally sized, non-empty point sets.')

    est_center_x = sum(point[0] for point in estimated_points) / len(estimated_points)
    est_center_y = sum(point[1] for point in estimated_points) / len(estimated_points)
    gt_center_x = sum(point[0] for point in ground_truth_points) / len(ground_truth_points)
    gt_center_y = sum(point[1] for point in ground_truth_points) / len(ground_truth_points)

    dot = 0.0
    cross = 0.0
    for estimated, ground_truth in zip(estimated_points, ground_truth_points):
        est_x = estimated[0] - est_center_x
        est_y = estimated[1] - est_center_y
        gt_x = ground_truth[0] - gt_center_x
        gt_y = ground_truth[1] - gt_center_y
        dot += est_x * gt_x + est_y * gt_y
        cross += est_x * gt_y - est_y * gt_x

    yaw = math.atan2(cross, dot) if abs(dot) + abs(cross) > 1e-12 else 0.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    translation_x = gt_center_x - (cos_yaw * est_center_x - sin_yaw * est_center_y)
    translation_y = gt_center_y - (sin_yaw * est_center_x + cos_yaw * est_center_y)
    return {
        'yaw_rad': yaw,
        'yaw_deg': math.degrees(yaw),
        'translation_x_m': translation_x,
        'translation_y_m': translation_y,
    }


def apply_se2(x: float, y: float, transform: Dict[str, float]) -> Tuple[float, float]:
    yaw = transform['yaw_rad']
    return (
        math.cos(yaw) * x - math.sin(yaw) * y + transform['translation_x_m'],
        math.sin(yaw) * x + math.cos(yaw) * y + transform['translation_y_m'],
    )


@dataclass
class WaypointResult:
    index: int
    label: str
    stamp_ns: int
    gt_x: float
    gt_y: float
    gt_yaw_deg: Optional[float]
    est_x: float
    est_y: float
    est_yaw_deg: float
    sample_count: int
    position_stability_std_m: float
    yaw_stability_std_deg: float
    raw_error_m: float
    aligned_x: Optional[float] = None
    aligned_y: Optional[float] = None
    aligned_yaw_deg: Optional[float] = None
    aligned_error_m: Optional[float] = None
    yaw_error_deg: Optional[float] = None


class WaypointAccuracyNode(Node):
    def __init__(self):
        super().__init__('waypoint_accuracy_monitor')

        self.waypoints_file = self.declare_parameter('waypoints_file', '').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.lookup_timeout_sec = float(self.declare_parameter('lookup_timeout_sec', 0.15).value)
        self.check_period_sec = float(self.declare_parameter('check_period_sec', 0.1).value)
        self.evaluation_delay_sec = float(self.declare_parameter('evaluation_delay_sec', 0.25).value)
        self.settling_window_sec = max(
            0.0,
            float(self.declare_parameter('settling_window_sec', 1.0).value),
        )
        self.window_sample_count = max(
            1,
            int(self.declare_parameter('window_sample_count', 11).value),
        )
        self.min_window_samples = max(
            1,
            int(self.declare_parameter('min_window_samples', 5).value),
        )
        self.alignment_mode = str(self.declare_parameter('alignment_mode', 'se2').value).lower()
        self.output_path = self.declare_parameter('output_path', '').value
        self.reference_frame_override = self.declare_parameter('reference_frame', '').value

        if not self.waypoints_file:
            raise ValueError('Parameter waypoints_file must be set.')
        if self.alignment_mode not in {'se2', 'none'}:
            raise ValueError("alignment_mode must be either 'se2' or 'none'.")
        if self.min_window_samples > self.window_sample_count:
            raise ValueError('min_window_samples cannot exceed window_sample_count.')

        self.tf_buffer = Buffer(cache_time=Duration(seconds=600.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.reference_frame, self.waypoints = self.load_waypoints(self.waypoints_file)
        if self.reference_frame_override:
            self.reference_frame = self.reference_frame_override

        self.next_waypoint_index = 0
        self.results: List[WaypointResult] = []
        self.failed_waypoints = []
        self.summary_written = False

        self.create_timer(self.check_period_sec, self.timer_callback)
        self.get_logger().info(
            f'Waypoint accuracy monitor started for {len(self.waypoints)} marks using '
            f'{self.reference_frame} -> {self.base_frame}; '
            f'{self.settling_window_sec:.2f}s/{self.window_sample_count} sample window, '
            f'alignment={self.alignment_mode}.'
        )

    @staticmethod
    def load_waypoints(path: str):
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)

        marks = sorted(data.get('marks', []), key=lambda mark: int(mark['stamp_ns']))
        if not marks:
            raise ValueError(f'No waypoint marks found in {path}.')
        for mark in marks:
            for required_key in ('stamp_ns', 'index', 'label', 'x', 'y'):
                if required_key not in mark:
                    raise ValueError(f"Waypoint is missing required field '{required_key}': {mark}")
        return data.get('frame', 'map'), marks

    def timer_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return

        post_window_ns = int(self.settling_window_sec * 0.5 * 1e9)
        delay_ns = int(self.evaluation_delay_sec * 1e9)
        while self.next_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.next_waypoint_index]
            waypoint_ready_ns = int(waypoint['stamp_ns']) + post_window_ns + delay_ns
            if waypoint_ready_ns > now_ns:
                break

            self.evaluate_waypoint(waypoint)
            self.next_waypoint_index += 1

        if not self.summary_written and self.next_waypoint_index >= len(self.waypoints):
            self.write_summary()

    def evaluate_waypoint(self, waypoint) -> None:
        stamp_ns = int(waypoint['stamp_ns'])
        samples = []
        failures = []

        for sample_stamp_ns in self.sample_timestamps(stamp_ns):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.base_frame,
                    self.time_from_ns(sample_stamp_ns),
                    timeout=Duration(seconds=self.lookup_timeout_sec),
                )
                samples.append(
                    (
                        float(transform.transform.translation.x),
                        float(transform.transform.translation.y),
                        yaw_from_quaternion(transform.transform.rotation),
                    )
                )
            except TransformException as exc:
                failures.append(str(exc))

        if len(samples) < self.min_window_samples:
            reason = (
                f'Only {len(samples)}/{self.window_sample_count} TF samples were available; '
                f'{self.min_window_samples} required.'
            )
            if failures:
                reason += f' Last TF error: {failures[-1]}'
            self.failed_waypoints.append(
                {
                    'index': int(waypoint['index']),
                    'label': str(waypoint['label']),
                    'stamp_ns': stamp_ns,
                    'reason': reason,
                }
            )
            self.get_logger().warning(f"Could not evaluate waypoint {waypoint['label']}: {reason}")
            return

        x_values = [sample[0] for sample in samples]
        y_values = [sample[1] for sample in samples]
        yaw_values = [sample[2] for sample in samples]
        est_x = statistics.median(x_values)
        est_y = statistics.median(y_values)
        est_yaw = circular_mean(yaw_values)
        gt_x = float(waypoint['x'])
        gt_y = float(waypoint['y'])
        gt_yaw_deg = self.get_gt_yaw_deg(waypoint)

        result = WaypointResult(
            index=int(waypoint['index']),
            label=str(waypoint['label']),
            stamp_ns=stamp_ns,
            gt_x=gt_x,
            gt_y=gt_y,
            gt_yaw_deg=gt_yaw_deg,
            est_x=est_x,
            est_y=est_y,
            est_yaw_deg=math.degrees(est_yaw),
            sample_count=len(samples),
            position_stability_std_m=math.hypot(
                statistics.pstdev(x_values) if len(x_values) > 1 else 0.0,
                statistics.pstdev(y_values) if len(y_values) > 1 else 0.0,
            ),
            yaw_stability_std_deg=math.degrees(circular_std(yaw_values)),
            raw_error_m=math.hypot(est_x - gt_x, est_y - gt_y),
        )
        self.results.append(result)
        self.get_logger().info(
            f'waypoint {result.label} raw est=({est_x:+.2f}, {est_y:+.2f}) '
            f'gt=({gt_x:+.2f}, {gt_y:+.2f}), raw error={result.raw_error_m:.3f}m, '
            f'stability={result.position_stability_std_m:.3f}m'
        )

    def sample_timestamps(self, center_stamp_ns: int) -> List[int]:
        if self.window_sample_count == 1 or self.settling_window_sec == 0.0:
            return [center_stamp_ns]
        start_ns = center_stamp_ns - int(self.settling_window_sec * 0.5 * 1e9)
        step_ns = int(self.settling_window_sec * 1e9 / (self.window_sample_count - 1))
        return [start_ns + index * step_ns for index in range(self.window_sample_count)]

    @staticmethod
    def get_gt_yaw_deg(waypoint) -> Optional[float]:
        if 'yaw_deg' in waypoint:
            return float(waypoint['yaw_deg'])
        if 'yaw' in waypoint:
            return math.degrees(float(waypoint['yaw']))
        return None

    def apply_alignment(self) -> Dict[str, object]:
        identity = {
            'yaw_rad': 0.0,
            'yaw_deg': 0.0,
            'translation_x_m': 0.0,
            'translation_y_m': 0.0,
        }
        if not self.results or self.alignment_mode == 'none':
            alignment = identity
        else:
            alignment = fit_se2(
                [(result.est_x, result.est_y) for result in self.results],
                [(result.gt_x, result.gt_y) for result in self.results],
            )

        for result in self.results:
            result.aligned_x, result.aligned_y = apply_se2(result.est_x, result.est_y, alignment)
            result.aligned_yaw_deg = math.degrees(
                normalize_angle(math.radians(result.est_yaw_deg) + alignment['yaw_rad'])
            )
            result.aligned_error_m = math.hypot(
                result.aligned_x - result.gt_x,
                result.aligned_y - result.gt_y,
            )
            if result.gt_yaw_deg is not None:
                result.yaw_error_deg = abs(
                    math.degrees(
                        normalize_angle(
                            math.radians(result.aligned_yaw_deg - result.gt_yaw_deg)
                        )
                    )
                )

        return {
            'mode': self.alignment_mode,
            'description': (
                'Least-squares rigid 2D transform from the SLAM map frame to the surveyed '
                'waypoint frame; no scale fitting.'
                if self.alignment_mode == 'se2'
                else 'No coordinate-frame alignment.'
            ),
            **alignment,
        }

    def revisit_metrics(self) -> List[Dict[str, object]]:
        grouped: Dict[str, List[WaypointResult]] = {}
        for result in self.results:
            grouped.setdefault(result.label, []).append(result)

        revisits = []
        for label, visits in grouped.items():
            if len(visits) < 2:
                continue
            first = visits[0]
            last = visits[-1]
            revisits.append(
                {
                    'label': label,
                    'first_index': first.index,
                    'last_index': last.index,
                    'elapsed_sec': (last.stamp_ns - first.stamp_ns) / 1e9,
                    'position_drift_m': math.hypot(
                        last.aligned_x - first.aligned_x,
                        last.aligned_y - first.aligned_y,
                    ),
                    'yaw_drift_deg': abs(
                        math.degrees(
                            normalize_angle(
                                math.radians(last.aligned_yaw_deg - first.aligned_yaw_deg)
                            )
                        )
                    ),
                }
            )
        return revisits

    def write_summary(self) -> None:
        if self.summary_written:
            return
        self.summary_written = True

        alignment = self.apply_alignment()
        raw_metrics = error_metrics([result.raw_error_m for result in self.results])
        aligned_metrics = error_metrics(
            [result.aligned_error_m for result in self.results if result.aligned_error_m is not None]
        )
        yaw_errors = [
            result.yaw_error_deg for result in self.results if result.yaw_error_deg is not None
        ]
        yaw_metrics = error_metrics(yaw_errors)
        yaw_metrics = {key.replace('_m', '_deg'): value for key, value in yaw_metrics.items()}
        revisits = self.revisit_metrics()

        summary = {
            'waypoints_file': self.waypoints_file,
            'reference_frame': self.reference_frame,
            'base_frame': self.base_frame,
            'evaluated_count': len(self.results),
            'failed_count': len(self.failed_waypoints),
            'settling_window_sec': self.settling_window_sec,
            'window_sample_count': self.window_sample_count,
            'alignment': alignment,
            'raw_position_metrics': raw_metrics,
            'aligned_position_metrics': aligned_metrics,
            'aligned_yaw_metrics': yaw_metrics,
            'revisit_count': len(revisits),
            'revisits': revisits,
            # Backwards-compatible top-level values use the selected alignment mode.
            'mae_m': aligned_metrics['mae_m'],
            'rmse_m': aligned_metrics['rmse_m'],
            'median_m': aligned_metrics['median_m'],
            'p95_m': aligned_metrics['p95_m'],
            'max_error_m': aligned_metrics['max_m'],
            'results': [asdict(result) for result in self.results],
            'failed_waypoints': self.failed_waypoints,
        }

        if self.output_path:
            output_dir = os.path.dirname(self.output_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            with open(self.output_path, 'w', encoding='utf-8') as handle:
                json.dump(summary, handle, indent=2, allow_nan=False)

        if self.results:
            self.get_logger().info(
                'Waypoint evaluation complete: '
                f'{len(self.results)} marks, aligned MAE={aligned_metrics["mae_m"]:.3f}m, '
                f'RMSE={aligned_metrics["rmse_m"]:.3f}m, '
                f'p95={aligned_metrics["p95_m"]:.3f}m, '
                f'max={aligned_metrics["max_m"]:.3f}m.'
            )
        else:
            self.get_logger().warning('Waypoint evaluation finished without successful lookups.')

        if self.output_path:
            self.get_logger().info(f'Wrote waypoint evaluation to {self.output_path}')

    @staticmethod
    def time_from_ns(stamp_ns: int) -> Time:
        return Time.from_msg(
            TimeMsg(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointAccuracyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.summary_written and (node.results or node.failed_waypoints):
            node.write_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
