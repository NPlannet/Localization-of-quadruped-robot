import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

        self.bridge = CvBridge()

        self.save_dir = os.path.join(
            os.getcwd(),
            'saved_images'
        )

        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info("Image Saver Node gestartet.")

    def listener_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        timestamp = datetime.now().strftime("%Y/%m/%d_%H:%M:%S_%f")
        filename = f"{self.save_dir}/img_{timestamp}.png"

        cv2.imwrite(filename, cv_img)

        self.get_logger().info(f"Bild gespeichert: {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()