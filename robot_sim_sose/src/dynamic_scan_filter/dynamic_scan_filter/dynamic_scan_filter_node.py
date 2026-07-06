import copy
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


UNCERTAIN = 'uncertain'
MOVING = 'moving'
STATIC_TRUSTED = 'static_trusted'


@dataclass
class ScanPoint:
    index: int
    range_value: float
    angle: float
    x: float
    y: float


@dataclass
class ClusterObservation:
    beam_indices: List[int]
    points_scan: List[Tuple[float, float]]
    centroid_scan: Tuple[float, float]
    centroid_tracking: Tuple[float, float]
    span: float
    mean_range: float
    trackable: bool
    state: str = STATIC_TRUSTED
    track_id: Optional[int] = None


@dataclass
class Track:
    track_id: int
    centroid_tracking: Tuple[float, float]
    last_timestamp_sec: float
    span: float
    state: str = UNCERTAIN
    moving_hits: int = 0
    static_hits: int = 1
    missed_frames: int = 0
    static_since_sec: Optional[float] = None
    centroid_history: List[Tuple[float, Tuple[float, float]]] = field(default_factory=list)


class DynamicScanFilterNode(Node):
    def __init__(self):
        super().__init__('dynamic_scan_filter')

        self.input_scan_topic = self.declare_parameter('input_scan_topic', '/scan').value
        self.output_scan_topic = self.declare_parameter('output_scan_topic', '/scan_filtered').value
        self.output_reliability = self.declare_parameter('output_reliability', 'reliable').value
        self.marker_topic = self.declare_parameter(
            'marker_topic',
            '/dynamic_scan_filter/cluster_markers',
        ).value
        self.tracking_frame = self.declare_parameter('tracking_frame', 'odom').value
        self.min_valid_range = float(self.declare_parameter('min_valid_range', 0.05).value)
        self.cluster_distance_base = float(self.declare_parameter('cluster_distance_base', 0.08).value)
        self.cluster_distance_scale = float(self.declare_parameter('cluster_distance_scale', 2.0).value)
        self.min_cluster_points = int(self.declare_parameter('min_cluster_points', 3).value)
        self.max_trackable_cluster_span = float(
            self.declare_parameter('max_trackable_cluster_span', 1.5).value
        )
        self.association_distance = float(self.declare_parameter('association_distance', 0.35).value)
                #This subtracts a small allowed wobble before motion is measured. If a cluster shifts by less than this, the code treats it as zero movement.
        self.motion_jitter_tolerance = float(self.declare_parameter('motion_jitter_tolerance', 0.02).value) 
        # Time window for motion history. Clusters are saved for 1 seconds, then compared to the new one for speed calculation
        self.motion_history_window_sec = float(self.declare_parameter('motion_history_window_sec', 1.0).value)
        self.motion_history_min_age_sec = float(self.declare_parameter('motion_history_min_age_sec', 1.0).value)
        self.range_adjustment_enabled = bool(self.declare_parameter('range_adjustment_enabled', True).value)
        self.range_adjustment_reference_range = float(
            self.declare_parameter('range_adjustment_reference_range', 1.0).value
        )
        self.range_adjustment_min_range = float(
            self.declare_parameter('range_adjustment_min_range', 0.2).value
        )
        self.range_adjustment_min_scale = float(
            self.declare_parameter('range_adjustment_min_scale', 0.7).value
        )
        self.range_adjustment_max_scale = float(
            self.declare_parameter('range_adjustment_max_scale', 1.3).value
        )
        # Most important parameters
        self.moving_speed_threshold = float(self.declare_parameter('moving_speed_threshold', 0.1).value)
        self.static_speed_threshold = float(self.declare_parameter('static_speed_threshold', 0.06).value)
        self.moving_confirmations = int(self.declare_parameter('moving_confirmations', 4).value)
        self.static_confirmations = int(self.declare_parameter('static_confirmations', 20).value)
        self.static_release_delay_sec = float(self.declare_parameter('static_release_delay_sec', 3.0).value)
        self.max_missed_frames = int(self.declare_parameter('max_missed_frames', 4).value)
        self.filter_unconfirmed = bool(self.declare_parameter('filter_unconfirmed', True).value)
        self.transform_timeout_sec = float(self.declare_parameter('transform_timeout_sec', 0.05).value)

        # marker settings on map
        self.marker_lifetime = Duration(seconds=float(self.declare_parameter('marker_lifetime_sec', 0.5).value))
        self.cluster_line_width = float(self.declare_parameter('cluster_line_width', 0.03).value)
        self.centroid_marker_size = float(self.declare_parameter('centroid_marker_size', 0.10).value)
        self.label_height = float(self.declare_parameter('label_height', 0.20).value)
        self.label_text_size = float(self.declare_parameter('label_text_size', 0.12).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        output_scan_qos = self.create_output_scan_qos()
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.output_scan_topic,
            output_scan_qos,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1
        self.last_tf_warning_ns = 0
        self.last_stats_log_ns = 0

        self.get_logger().info(
            'Dynamic scan filter started: '
            f'{self.input_scan_topic} -> {self.output_scan_topic}, tracking in {self.tracking_frame}, '
            f'markers on {self.marker_topic}.'
        )

    def create_output_scan_qos(self) -> QoSProfile:
        reliability_name = str(self.output_reliability).strip().lower()
        if reliability_name == 'best_effort':
            reliability = QoSReliabilityPolicy.BEST_EFFORT
        else:
            if reliability_name not in {'reliable', 'system_default'}:
                self.get_logger().warning(
                    f"Unknown output_reliability '{self.output_reliability}', using reliable."
                )
            reliability = (
                QoSReliabilityPolicy.SYSTEM_DEFAULT
                if reliability_name == 'system_default'
                else QoSReliabilityPolicy.RELIABLE
            )

        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
        )

    def scan_callback(self, msg: LaserScan) -> None:
        points = self.extract_points(msg)
        if not points:
            self.scan_pub.publish(msg)
            self.clear_markers(msg)
            return

        transform = self.lookup_transform(msg)
        if transform is None:
            self.scan_pub.publish(msg)
            self.clear_markers(msg)
            return

        observations = self.build_clusters(points, msg.angle_increment, transform)
        timestamp_sec = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        self.update_tracks(observations, timestamp_sec)
        filtered_scan, filtered_beams = self.build_filtered_scan(msg, observations)
        self.scan_pub.publish(filtered_scan)
        self.publish_markers(msg, observations)
        self.log_stats(observations, filtered_beams)

    def extract_points(self, msg: LaserScan) -> List[ScanPoint]:
        points: List[ScanPoint] = []
        max_valid_range = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0.0 else float('inf')

        for index, range_value in enumerate(msg.ranges):
            if not math.isfinite(range_value):
                continue
            if range_value < self.min_valid_range or range_value > max_valid_range:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            points.append(
                ScanPoint(
                    index=index,
                    range_value=range_value,
                    angle=angle,
                    x=range_value * math.cos(angle),
                    y=range_value * math.sin(angle),
                )
            )

        return points

    def lookup_transform(self, msg: LaserScan):
        stamp = Time.from_msg(msg.header.stamp)
        timeout = Duration(seconds=self.transform_timeout_sec)

        try:
            return self.tf_buffer.lookup_transform(
                self.tracking_frame,
                msg.header.frame_id,
                stamp,
                timeout=timeout,
            )
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(
                    self.tracking_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=timeout,
                )
            except TransformException as exc:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self.last_tf_warning_ns > 5_000_000_000:
                    self.last_tf_warning_ns = now_ns
                    self.get_logger().warning(
                        f'Could not transform {msg.header.frame_id} into {self.tracking_frame}: {exc}'
                    )
                return None

    def build_clusters(self, points: List[ScanPoint], angle_increment: float, transform) -> List[ClusterObservation]:
        observations: List[ClusterObservation] = []
        current_cluster: List[ScanPoint] = []

        for point in points:
            if not current_cluster:
                current_cluster.append(point)
                continue

            previous = current_cluster[-1]
            contiguous = point.index == previous.index + 1
            close_enough = self.points_belong_to_same_cluster(previous, point, angle_increment)

            if contiguous and close_enough:
                current_cluster.append(point)
                continue

            observation = self.finalize_cluster(current_cluster, transform)
            if observation is not None:
                observations.append(observation)
            current_cluster = [point]

        observation = self.finalize_cluster(current_cluster, transform)
        if observation is not None:
            observations.append(observation)

        return observations

    def points_belong_to_same_cluster(
        self,
        previous: ScanPoint,
        current: ScanPoint,
        angle_increment: float,
    ) -> bool:
        distance = math.hypot(current.x - previous.x, current.y - previous.y)
        threshold = self.cluster_distance_base + (
            self.cluster_distance_scale * min(previous.range_value, current.range_value) * abs(angle_increment)
        )
        return distance <= threshold

    def finalize_cluster(self, cluster_points: List[ScanPoint], transform) -> Optional[ClusterObservation]:
        if len(cluster_points) < self.min_cluster_points:
            return None

        centroid_scan = (
            sum(point.x for point in cluster_points) / len(cluster_points),
            sum(point.y for point in cluster_points) / len(cluster_points),
        )
        centroid_tracking = self.transform_point(centroid_scan, transform)
        span = math.hypot(
            cluster_points[-1].x - cluster_points[0].x,
            cluster_points[-1].y - cluster_points[0].y,
        )
        mean_range = sum(point.range_value for point in cluster_points) / len(cluster_points)
        beam_indices = [point.index for point in cluster_points]
        trackable = span <= self.max_trackable_cluster_span

        return ClusterObservation(
            beam_indices=beam_indices,
            points_scan=[(point.x, point.y) for point in cluster_points],
            centroid_scan=centroid_scan,
            centroid_tracking=centroid_tracking,
            span=span,
            mean_range=mean_range,
            trackable=trackable,
        )

    def transform_point(self, point: Tuple[float, float], transform) -> Tuple[float, float]:
        x, y = point
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rot00 = 1.0 - 2.0 * (yy + zz)
        rot01 = 2.0 * (xy - wz)
        rot10 = 2.0 * (xy + wz)
        rot11 = 1.0 - 2.0 * (xx + zz)

        transformed_x = translation.x + rot00 * x + rot01 * y
        transformed_y = translation.y + rot10 * x + rot11 * y
        return transformed_x, transformed_y

    def update_tracks(self, observations: List[ClusterObservation], timestamp_sec: float) -> None:
        matched_track_ids = set()
        matched_observation_indices = set()
        candidates = []

        for observation_index, observation in enumerate(observations):
            if not observation.trackable:
                observation.state = STATIC_TRUSTED
                continue

            for track_id, track in self.tracks.items():
                distance = math.hypot(
                    observation.centroid_tracking[0] - track.centroid_tracking[0],
                    observation.centroid_tracking[1] - track.centroid_tracking[1],
                )
                if distance <= self.association_distance:
                    candidates.append((distance, observation_index, track_id))

        candidates.sort(key=lambda item: item[0])

        for distance, observation_index, track_id in candidates:
            if observation_index in matched_observation_indices or track_id in matched_track_ids:
                continue

            observation = observations[observation_index]
            track = self.tracks[track_id]
            self.update_track(track, observation, distance, timestamp_sec)
            observation.track_id = track_id
            observation.state = track.state
            matched_observation_indices.add(observation_index)
            matched_track_ids.add(track_id)

        for observation_index, observation in enumerate(observations):
            if observation_index in matched_observation_indices or not observation.trackable:
                continue

            track_id = self.next_track_id
            self.next_track_id += 1
            self.tracks[track_id] = Track(
                track_id=track_id,
                centroid_tracking=observation.centroid_tracking,
                last_timestamp_sec=timestamp_sec,
                span=observation.span,
                centroid_history=[(timestamp_sec, observation.centroid_tracking)],
            )
            observation.track_id = track_id
            observation.state = self.tracks[track_id].state
            matched_track_ids.add(track_id)

        for track_id in list(self.tracks.keys()):
            if track_id in matched_track_ids:
                self.tracks[track_id].missed_frames = 0
                continue

            self.tracks[track_id].missed_frames += 1
            if self.tracks[track_id].missed_frames > self.max_missed_frames:
                del self.tracks[track_id]

    def update_track(
        self,
        track: Track,
        observation: ClusterObservation,
        match_distance: float,
        timestamp_sec: float,
    ) -> None:
        del match_distance

        reference_timestamp_sec, reference_centroid = self.get_motion_reference(track, timestamp_sec)
        dt = max(timestamp_sec - reference_timestamp_sec, 1e-3)
        raw_distance = math.hypot(
            observation.centroid_tracking[0] - reference_centroid[0],
            observation.centroid_tracking[1] - reference_centroid[1],
        )
        adjusted_distance = self.adjust_motion_distance_for_range(raw_distance, observation.mean_range)
        effective_distance = max(0.0, adjusted_distance - self.motion_jitter_tolerance)
        speed = effective_distance / dt
        was_moving = track.state == MOVING
        was_static = track.state == STATIC_TRUSTED

        track.centroid_tracking = observation.centroid_tracking
        track.last_timestamp_sec = timestamp_sec
        track.span = observation.span
        self.append_centroid_history(track, timestamp_sec, observation.centroid_tracking)

        moving_candidate = speed >= self.moving_speed_threshold
        static_candidate = speed <= self.static_speed_threshold

        if moving_candidate:
            track.moving_hits += 1
            track.static_hits = 0
            track.static_since_sec = None
            track.state = MOVING if was_moving or track.moving_hits >= self.moving_confirmations else UNCERTAIN
            return

        if static_candidate:
            track.static_hits += 1
            track.moving_hits = 0
            if was_moving:
                if track.static_since_sec is None:
                    track.static_since_sec = timestamp_sec
                # Keep filtering a previously moving object until it has stayed
                # quiet long enough to be trusted as static again.
                if timestamp_sec - track.static_since_sec >= self.static_release_delay_sec:
                    track.state = STATIC_TRUSTED
                else:
                    track.state = MOVING
            elif was_static or track.static_hits >= self.static_confirmations:
                track.state = STATIC_TRUSTED
                track.static_since_sec = None
            else:
                track.state = UNCERTAIN
                track.static_since_sec = None
            return

        track.moving_hits = 0
        track.static_hits = 0
        track.static_since_sec = None
        if was_moving:
            track.state = MOVING
        elif was_static:
            track.state = STATIC_TRUSTED
        else:
            track.state = UNCERTAIN

    def adjust_motion_distance_for_range(self, raw_distance: float, mean_range: float) -> float:
        if not self.range_adjustment_enabled:
            return raw_distance

        safe_range = max(mean_range, self.range_adjustment_min_range)
        if not math.isfinite(safe_range) or safe_range <= 0.0:
            return raw_distance

        scale = self.range_adjustment_reference_range / safe_range
        scale = min(max(scale, self.range_adjustment_min_scale), self.range_adjustment_max_scale)
        return raw_distance * scale

    def get_motion_reference(
        self,
        track: Track,
        timestamp_sec: float,
    ) -> Tuple[float, Tuple[float, float]]:
        if not track.centroid_history:
            return track.last_timestamp_sec, track.centroid_tracking

        min_age_sec = min(self.motion_history_min_age_sec, self.motion_history_window_sec)
        eligible_samples = [
            sample for sample in track.centroid_history
            if timestamp_sec - sample[0] >= min_age_sec
        ]
        if not eligible_samples:
            return track.last_timestamp_sec, track.centroid_tracking

        target_age_sec = self.motion_history_window_sec
        return min(
            eligible_samples,
            key=lambda sample: abs((timestamp_sec - sample[0]) - target_age_sec),
        )

    def append_centroid_history(
        self,
        track: Track,
        timestamp_sec: float,
        centroid_tracking: Tuple[float, float],
    ) -> None:
        track.centroid_history.append((timestamp_sec, centroid_tracking))
        keep_after_sec = timestamp_sec - max(
            self.motion_history_window_sec + self.motion_history_min_age_sec,
            self.motion_history_window_sec + 0.5,
        )
        track.centroid_history = [
            sample for sample in track.centroid_history
            if sample[0] >= keep_after_sec
        ]

    def build_filtered_scan(
        self,
        msg: LaserScan,
        observations: List[ClusterObservation],
    ) -> Tuple[LaserScan, int]:
        filtered_scan = copy.deepcopy(msg)
        filtered_beam_count = 0

        for observation in observations:
            if not self.should_filter_observation(observation):
                continue

            for beam_index in observation.beam_indices:
                filtered_scan.ranges[beam_index] = float('inf')
                filtered_beam_count += 1

        return filtered_scan, filtered_beam_count

    def should_filter_observation(self, observation: ClusterObservation) -> bool:
        if observation.state == MOVING:
            return True
        if self.filter_unconfirmed and observation.state != STATIC_TRUSTED:
            return True
        return False

    def log_stats(self, observations: List[ClusterObservation], filtered_beams: int) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_stats_log_ns < 5_000_000_000:
            return

        self.last_stats_log_ns = now_ns
        moving_clusters = sum(1 for observation in observations if observation.state == MOVING)
        trackable_clusters = sum(1 for observation in observations if observation.trackable)
        self.get_logger().info(
            'Published filtered scan with '
            f'{filtered_beams} beams removed from {moving_clusters}/{trackable_clusters} trackable clusters.'
        )

    def publish_markers(self, msg: LaserScan, observations: List[ClusterObservation]) -> None:
        marker_array = MarkerArray()
        marker_array.markers.append(self.make_delete_all_marker(msg))

        marker_id = 0
        for observation in observations:
            color = self.get_marker_color(observation)
            label = self.get_marker_label(observation)

            line_marker = Marker()
            line_marker.header = msg.header
            line_marker.ns = 'cluster_lines'
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = self.cluster_line_width
            line_marker.color.r = color[0]
            line_marker.color.g = color[1]
            line_marker.color.b = color[2]
            line_marker.color.a = color[3]
            line_marker.lifetime = self.marker_lifetime.to_msg()
            line_marker.points = [self.make_point(x, y, 0.03) for x, y in observation.points_scan]
            marker_array.markers.append(line_marker)

            centroid_marker = Marker()
            centroid_marker.header = msg.header
            centroid_marker.ns = 'cluster_centroids'
            centroid_marker.id = marker_id
            marker_id += 1
            centroid_marker.type = Marker.SPHERE
            centroid_marker.action = Marker.ADD
            centroid_marker.pose.orientation.w = 1.0
            centroid_marker.pose.position = self.make_point(
                observation.centroid_scan[0],
                observation.centroid_scan[1],
                0.05,
            )
            centroid_marker.scale.x = self.centroid_marker_size
            centroid_marker.scale.y = self.centroid_marker_size
            centroid_marker.scale.z = self.centroid_marker_size
            centroid_marker.color.r = color[0]
            centroid_marker.color.g = color[1]
            centroid_marker.color.b = color[2]
            centroid_marker.color.a = min(1.0, color[3] + 0.15)
            centroid_marker.lifetime = self.marker_lifetime.to_msg()
            marker_array.markers.append(centroid_marker)

            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = 'cluster_labels'
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.orientation.w = 1.0
            text_marker.pose.position = self.make_point(
                observation.centroid_scan[0],
                observation.centroid_scan[1],
                self.label_height,
            )
            text_marker.scale.z = self.label_text_size
            text_marker.color.r = color[0]
            text_marker.color.g = color[1]
            text_marker.color.b = color[2]
            text_marker.color.a = 1.0
            text_marker.text = label
            text_marker.lifetime = self.marker_lifetime.to_msg()
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def clear_markers(self, msg: LaserScan) -> None:
        marker_array = MarkerArray()
        marker_array.markers.append(self.make_delete_all_marker(msg))
        self.marker_pub.publish(marker_array)

    def make_delete_all_marker(self, msg: LaserScan) -> Marker:
        marker = Marker()
        marker.header = msg.header
        marker.action = Marker.DELETEALL
        return marker

    def get_marker_color(self, observation: ClusterObservation) -> Tuple[float, float, float, float]:
        if not observation.trackable:
            return 0.40, 0.65, 1.00, 0.85
        if observation.state == MOVING:
            return 0.95, 0.20, 0.20, 0.95
        if observation.state == UNCERTAIN:
            return 1.00, 0.75, 0.10, 0.90
        return 0.20, 0.85, 0.35, 0.90

    def get_marker_label(self, observation: ClusterObservation) -> str:
        if not observation.trackable:
            return f'cluster span={observation.span:.2f}m'

        track_label = '?' if observation.track_id is None else str(observation.track_id)
        if observation.state == MOVING and observation.track_id is not None:
            track = self.tracks.get(observation.track_id)
            if track is not None and track.static_since_sec is not None:
                remaining_sec = max(
                    0.0,
                    self.static_release_delay_sec - (track.last_timestamp_sec - track.static_since_sec),
                )
                return f'{track_label}: moving hold={remaining_sec:.1f}s'
        return f'{track_label}: {observation.state}'

    def make_point(self, x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point


def main(args=None):
    rclpy.init(args=args)
    node = DynamicScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
