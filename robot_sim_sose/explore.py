from math import radians, isfinite, hypot
import tf_transformations
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import os

from sensor_msgs.msg import LaserScan, Imu, Image
from PIL import Image as PILImage

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(OccupancyGrid,'/map', self.map_callback, 10)
        self.navigator = BasicNavigator()
        
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        
        self.lidar_ranges = None
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.imu_data = None
        self.camera_image = None
        self.map_data = None
        self.bridge = CvBridge()
        self.explore_timer = self.create_timer(5.0, self.explore_step)
        self.iteration = 0
        
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def scan_callback(self, msg):
        self.lidar_ranges = msg.ranges
        self.front_distance = get_range_at_angle(msg, 0)
        self.left_distance = get_range_at_angle(msg, 45)
        self.right_distance = get_range_at_angle(msg, -45)
        # self.get_logger().info(
        #     f"LIDAR DISTANCES: front={self.front_distance:.2f}, "
        #     f"left={self.left_distance:.2f}, right={self.right_distance:.2f}"
        # )
        
        
    def imu_callback(self, msg):
        self.imu_data = msg
        
    
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            self.camera_image = cv_image
            self.get_logger().info(f"Camera image received: {cv_image.shape}")
        except Exception as e:
            self.get_logger().error(f"Camera conversion failed: {e}")
            
    def map_callback(self, msg):
        self.map_info = msg.info

        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))


    def save_map_image(self):
        if self.map_data is not None:
            os.makedirs("maps", exist_ok=True)
            img = np.zeros_like(self.map_data, dtype=np.uint8)
        
            img[self.map_data == -1] = 128
            img[self.map_data == 0] = 255
            img[self.map_data == 100] = 0
        
            image = PILImage.fromarray(img)
            image.save(f"maps/map_{self.iteration:04d}.png")
            self.iteration += 1
            
    def explore_step(self):
        if self.map_data is None:
            return
        self.save_map_image()
        
        if not self.navigator.isTaskComplete():
            self.get_logger().info("Still navigating")
            return
        
        discover  = self.discover_next()
        if discover is None:
            self.get_logger().info("Nothing to discover found.")
            return
        x, y, _, = discover
        wx, wy = self.grid_to_world(x, y)
        self.get_logger().info(f"Navigating to grid=({x}, {y}) world=({wx:.2f}, {wy:.2f})")
        self.navigate_to(wx, wy)
        

        
    def navigate_to(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.navigator.goToPose(goal)


    def grid_to_world(self, gx, gy):
        resolution = self.map_info.resolution
        wx = gx * resolution + self.map_info.origin.position.x
        wy = gy * resolution + self.map_info.origin.position.y
        return wx, wy
    
    
    def discover_next(self, min_distance = 1.5):
        if self.map_data is None:
            return None
    
        options = []
        h, w = self.map_data.shape
    
        robot_gx = int((self.current_x - self.map_info.origin.position.x)/self.map_info.resolution)
        robot_gy = int((self.current_y - self.map_info.origin.position.y)/ self.map_info.resolution)
    
        for y in range(1, h - 1):
            for x in range(1, w - 1):
    
                if self.map_data[y, x] != 0:
                    continue
    
                neighbors = [
                    self.map_data[y + 1, x],
                    self.map_data[y - 1, x],
                    self.map_data[y, x + 1],
                    self.map_data[y, x - 1],
                ]
    
                if -1 in neighbors:
                    distance = (hypot(x - robot_gx, y - robot_gy)* self.map_info.resolution)
                    if distance < min_distance:
                        continue
                    options.append((x, y, distance))
    
        if not options:
            return None
    
        options.sort(key=lambda f: f[2])
        return options[0]


def dynamic_explore():
    rclpy.init()
    robot = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        robot.destroy_node()
        rclpy.shutdown()
    


def get_range_at_angle(scan, target_angle_deg, width_deg=10):
    target = radians(target_angle_deg)
    width = radians(width_deg)

    values = []

    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment

        if abs(angle - target) <= width / 2:
            if isfinite(distance):
                values.append(distance)

    if not values:
        return float('inf')

    return min(values)


if __name__ == '__main__':
    dynamic_explore()
