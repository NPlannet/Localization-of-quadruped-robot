import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class WaypointMarkerNode(Node):
    """Timestamp waypoint button presses with the robot's ROS clock."""

    def __init__(self) -> None:
        super().__init__('waypoint_marker')

        trigger_topic = str(
            self.declare_parameter(
                'trigger_topic', '/ground_truth/waypoint_trigger'
            ).value
        )
        marker_topic = str(
            self.declare_parameter(
                'marker_topic', '/ground_truth/waypoint'
            ).value
        )

        self.publisher = self.create_publisher(String, marker_topic, 10)
        self.subscription = self.create_subscription(
            Empty,
            trigger_topic,
            self.mark_waypoint,
            10,
        )
        self.mark_count = 0

        self.get_logger().info(
            f'Waypoint marker ready: publish std_msgs/msg/Empty to '
            f'{trigger_topic}; timestamp events are published on {marker_topic}.'
        )

    def mark_waypoint(self, _message: Empty) -> None:
        stamp_ns = int(self.get_clock().now().nanoseconds)
        if stamp_ns <= 0:
            self.get_logger().error(
                'ROS time is zero; the waypoint was not marked.'
            )
            return

        output = String()
        output.data = json.dumps(
            {'stamp_ns': stamp_ns},
            separators=(',', ':'),
        )
        self.publisher.publish(output)
        self.mark_count += 1
        self.get_logger().info(
            f'Marked waypoint {self.mark_count}: stamp_ns={stamp_ns}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
