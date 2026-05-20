from math import radians
from time import time, sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def move_forward(self, distance, speed=0.2):

        msg = Twist()
        msg.linear.x = speed
        duration = abs(distance / speed)
        start_time = time()

        while( time() - start_time < duration):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop()

    def rotate(self, degrees, angular_speed=0.5):

        rad = radians(degrees)
        msg = Twist()
        if (rad >= 0):
            msg.angular.z = angular_speed
        else:
            msg.angular.z = -angular_speed

        duration = abs(rad / angular_speed)

        start_time = time()

        while (time() - start_time < duration):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop()


def dummy_explore():
    rclpy.init()
    robot = RobotController()
    while(True):
        robot.move_forward(1.0)
        sleep(2)
        robot.rotate(90)
        sleep(2)
    

if __name__ == '__main__':
    dummy_explore()