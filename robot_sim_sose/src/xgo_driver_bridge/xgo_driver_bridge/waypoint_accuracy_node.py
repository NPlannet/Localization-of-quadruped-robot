import json
import math
import os
from dataclasses import asdict, dataclass
from typing import List

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class WaypointResult:
    index: int
    label: str
    stamp_ns: int
    gt_x: float
    gt_y: float
    est_x: float
    est_y: float
    error_m: float


class WaypointAccuracyNode(Node):
    def __init__(self):
        super().__init__('waypoint_accuracy_monitor')

        self.waypoints_file = self.declare_parameter('waypoints_file', '').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.lookup_timeout_sec = float(self.declare_parameter('lookup_timeout_sec', 0.15).value)
        self.check_period_sec = float(self.declare_parameter('check_period_sec', 0.1).value)
        self.evaluation_delay_sec = float(self.declare_parameter('evaluation_delay_sec', 0.25).value)
        self.output_path = self.declare_parameter('output_path', '').value
        self.reference_frame_override = self.declare_parameter('reference_frame', '').value

        if not self.waypoints_file:
            raise ValueError('Parameter waypoints_file must be set.')

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
            f'Waypoint accuracy monitor started for {len(self.waypoints)} marks '
            f'using {self.reference_frame} -> {self.base_frame}.'
        )

    def load_waypoints(self, path: str):
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)

        marks = sorted(data.get('marks', []), key=lambda mark: int(mark['stamp_ns']))
        frame = data.get('frame', 'map')
        return frame, marks

    def timer_callback(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return

        while self.next_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.next_waypoint_index]
            waypoint_ready_ns = int(waypoint['stamp_ns']) + int(self.evaluation_delay_sec * 1e9)
            if waypoint_ready_ns > now_ns:
                break

            self.evaluate_waypoint(waypoint)
            self.next_waypoint_index += 1

        if not self.summary_written and self.next_waypoint_index >= len(self.waypoints):
            self.summary_written = True
            self.write_summary()

    def evaluate_waypoint(self, waypoint) -> None:
        stamp_ns = int(waypoint['stamp_ns'])
        stamp = self.time_from_ns(stamp_ns)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
        except TransformException as exc:
            self.failed_waypoints.append(
                {
                    'index': int(waypoint['index']),
                    'label': str(waypoint['label']),
                    'stamp_ns': stamp_ns,
                    'reason': str(exc),
                }
            )
            self.get_logger().warning(
                f"Could not evaluate waypoint {waypoint['label']} at {stamp_ns}: {exc}"
            )
            return

        est_x = float(transform.transform.translation.x)
        est_y = float(transform.transform.translation.y)
        gt_x = float(waypoint['x'])
        gt_y = float(waypoint['y'])
        error_m = math.hypot(est_x - gt_x, est_y - gt_y)

        result = WaypointResult(
            index=int(waypoint['index']),
            label=str(waypoint['label']),
            stamp_ns=stamp_ns,
            gt_x=gt_x,
            gt_y=gt_y,
            est_x=est_x,
            est_y=est_y,
            error_m=error_m,
        )
        self.results.append(result)
        self.get_logger().info(
            f"waypoint {result.label} "
            f"gt=({gt_x:+.2f}, {gt_y:+.2f}) "
            f"est=({est_x:+.2f}, {est_y:+.2f}) "
            f"error={error_m:.3f} m"
        )

    def write_summary(self) -> None:
        if self.results:
            errors = [result.error_m for result in self.results]
            mae = sum(errors) / len(errors)
            rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
            max_error = max(errors)
        else:
            mae = float('nan')
            rmse = float('nan')
            max_error = float('nan')

        summary = {
            'waypoints_file': self.waypoints_file,
            'reference_frame': self.reference_frame,
            'base_frame': self.base_frame,
            'evaluated_count': len(self.results),
            'failed_count': len(self.failed_waypoints),
            'mae_m': mae,
            'rmse_m': rmse,
            'max_error_m': max_error,
            'results': [asdict(result) for result in self.results],
            'failed_waypoints': self.failed_waypoints,
        }

        if self.output_path:
            output_dir = os.path.dirname(self.output_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            with open(self.output_path, 'w', encoding='utf-8') as handle:
                json.dump(summary, handle, indent=2)

        if self.results:
            self.get_logger().info(
                'Waypoint evaluation complete: '
                f'{len(self.results)} marks, '
                f'MAE={mae:.3f} m, RMSE={rmse:.3f} m, max={max_error:.3f} m.'
            )
        else:
            self.get_logger().warning('Waypoint evaluation finished without any successful lookups.')

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
