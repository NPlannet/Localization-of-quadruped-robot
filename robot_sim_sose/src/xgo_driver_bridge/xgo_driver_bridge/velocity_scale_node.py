#!/usr/bin/env python3
"""Republishes /xgo/applied_vel scaled by a constant factor.

Used to test how systematically over/under-estimated commanded velocity
affects offline odometry and downstream SLAM accuracy. Does not touch
angular velocity direction/heading logic - only scales linear and
angular magnitude uniformly, matching what a miscalibrated speed
constant would do.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class VelocityScaleNode(Node):

    def __init__(self):
        super().__init__('velocity_scale_node')
        self.declare_parameter('input_topic', '/xgo/applied_vel')
        self.declare_parameter('output_topic', '/xgo/applied_vel_scaled')
        self.declare_parameter('scale', 1.0)

        self.scale = float(self.get_parameter('scale').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(TwistStamped, output_topic, 50)
        self.sub = self.create_subscription(
            TwistStamped, input_topic, self.on_twist, 50
        )
        self.get_logger().info(
            f'velocity_scale_node: {input_topic} -> {output_topic} '
            f'(scale={self.scale})'
        )

    def on_twist(self, msg: TwistStamped) -> None:
        out = TwistStamped()
        out.header = msg.header
        out.twist.linear.x = msg.twist.linear.x * self.scale
        out.twist.linear.y = msg.twist.linear.y * self.scale
        out.twist.linear.z = msg.twist.linear.z * self.scale
        out.twist.angular.x = msg.twist.angular.x * self.scale
        out.twist.angular.y = msg.twist.angular.y * self.scale
        out.twist.angular.z = msg.twist.angular.z * self.scale
        self.pub.publish(out)


def main():
    rclpy.init()
    node = VelocityScaleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()