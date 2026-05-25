from math import radians, atan2, cos, sin
from time import time, sleep

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations



class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.current_yaw = 0.0
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.current_yaw = yaw
        
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def move_forward(self, distance, speed=0.5):
        msg = Twist()
        msg.linear.x = speed
        duration = abs(distance / speed)
        start_time = time()

        while( time() - start_time < duration):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop()

    def rotate(self, degrees, angular_speed=0.5):
        target = radians(degrees)
        start = self.current_yaw
    
        msg = Twist()
        if target > 0:
            msg.angular.z = angular_speed
        else:
            msg.angular.z = -angular_speed
    
        while (abs(atan2(sin((self.current_yaw - start)), cos((self.current_yaw - start)))) < abs(target)):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
    
        self.stop()

def dummy_explore():
    rclpy.init()
    robot = RobotController()
    while(True):
        print("forward")
        robot.move_forward(1.0)
        sleep(1)
        print("rotate")
        robot.rotate(90)
        sleep(1)
    

if __name__ == '__main__':
    dummy_explore()