import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNormalizerNode(Node):
    """Normalize recorded LD19 scans for deterministic offline SLAM replay.

    The W1 bag contains 497-507 beams per revolution and occasionally reports
    a 0.20 s scan duration even though messages arrive every 0.10 s. Those
    per-beam timestamps overlap the following scan, which Cartographer rejects
    as out-of-order data. A fixed angular grid and zero per-beam time offsets
    are appropriate for this slow, offline 2D mapping experiment.
    """

    def __init__(self):
        super().__init__('scan_normalizer')

        self.input_topic = self.declare_parameter('input_scan_topic', '/scan').value
        self.output_topic = self.declare_parameter(
            'output_scan_topic',
            '/scan_normalized',
        ).value
        self.target_beam_count = max(
            2,
            int(self.declare_parameter('target_beam_count', 503).value),
        )
        self.target_angle_min = float(
            self.declare_parameter('target_angle_min', 0.0).value
        )
        self.target_angle_max = float(
            self.declare_parameter('target_angle_max', 2.0 * math.pi).value
        )
        self.normalized_scan_time = max(
            0.0,
            float(self.declare_parameter('normalized_scan_time', 0.1).value),
        )

        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, output_qos)
        self.get_logger().info(
            f'Scan normalizer started: {self.input_topic} -> {self.output_topic}, '
            f'{self.target_beam_count} fixed beams, zero per-beam time offsets.'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        if len(msg.ranges) < 2 or msg.angle_increment <= 0.0:
            self.get_logger().warning('Received an invalid LaserScan; forwarding unchanged.')
            self.scan_pub.publish(msg)
            return

        normalized = copy.deepcopy(msg)
        normalized.angle_min = self.target_angle_min
        normalized.angle_max = self.target_angle_max
        normalized.angle_increment = (
            self.target_angle_max - self.target_angle_min
        ) / (self.target_beam_count - 1)
        normalized.time_increment = 0.0
        normalized.scan_time = self.normalized_scan_time
        normalized.ranges = self.resample(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            normalized.angle_min,
            normalized.angle_increment,
        )
        if len(msg.intensities) == len(msg.ranges):
            normalized.intensities = self.resample(
                msg.intensities,
                msg.angle_min,
                msg.angle_increment,
                normalized.angle_min,
                normalized.angle_increment,
            )
        else:
            normalized.intensities = []

        self.scan_pub.publish(normalized)

    def resample(
        self,
        values,
        source_angle_min: float,
        source_angle_increment: float,
        target_angle_min: float,
        target_angle_increment: float,
    ):
        result = []
        last_index = len(values) - 1
        for target_index in range(self.target_beam_count):
            target_angle = target_angle_min + target_index * target_angle_increment
            source_position = (target_angle - source_angle_min) / source_angle_increment
            source_index = min(max(int(round(source_position)), 0), last_index)
            result.append(values[source_index])
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ScanNormalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
