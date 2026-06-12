import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO

import cv2
import os


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.model = YOLO('yolov8s.pt')
        self.bridge = CvBridge()
        self.device = self._resolve_device()
        self.frame_counter = 0

        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            Detection2DArray,
            '/detected_objects',
            10
        )
        self.get_logger().info(f'YOLO detector started on device: {self.device}')

    def _resolve_device(self):
        requested_device = os.environ.get('YOLO_DEVICE', 'auto').strip().lower()

        if requested_device in ('', 'auto'):
            return 'cuda' if torch.cuda.is_available() else 'cpu'

        if requested_device == 'cuda' and not torch.cuda.is_available():
            self.get_logger().warning(
                'YOLO_DEVICE=cuda requested, but CUDA is not available. Falling back to cpu.'
            )
            return 'cpu'

        return requested_device

    def callback(self, msg):
        self.frame_counter += 1

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, device=self.device, verbose=False)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for result in results:
            for box in result.boxes:
                det = Detection2D()
                det.header = msg.header

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox.center.position.x = (x1 + x2) / 2
                det.bbox.center.position.y = (y1 + y2) / 2
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(box.cls[0]))
                hyp.hypothesis.score = float(box.conf[0])
                det.results.append(hyp)

                detection_array.detections.append(det)

        self.pub.publish(detection_array)
        count = len(detection_array.detections)
        self.get_logger().info(f'{count} Objekte erkannt')
        if count > 0:
            annotated = results[0].plot()
            timestamp = self.get_clock().now().nanoseconds
            os.makedirs('/workspaces/robot_sim_sose/detections', exist_ok=True)
            cv2.imwrite(f'/workspaces/robot_sim_sose/detections/{timestamp}.jpg', annotated)
            self.get_logger().info(f'Bild gespeichert: detections/{timestamp}.jpg')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
