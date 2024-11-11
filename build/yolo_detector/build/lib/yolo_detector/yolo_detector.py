import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLOv10
import cv2

class YoloDetector(Node):
    """
    Class to detect objects using YOLO.
    """
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLOv10('yolov10n.pt')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, 'image', self.image_callback, 10)
        
    def image_callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = self.model(cv_image)
        annotated_frame = results[0].plot() 
        cv2.imshow('YOLOv10 Object Detection', annotated_frame)
        
if __name__ == '__main__':
    rclpy.init()
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    rclpy.shutdown()