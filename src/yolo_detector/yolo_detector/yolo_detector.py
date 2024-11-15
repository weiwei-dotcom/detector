import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# my_project/main.py
# import sys
# from pathlib import Path

# # 添加 yolov10 源代码目录到 sys.path
# sys.path.append(str(Path("/home/ww/Documents/yolov10/ultralytics/models").resolve()))

# 导入 YOLOv10 类并创建对象
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    """
    Class to detect objects using YOLO.
    """
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO('/home/wl/Documents/yolov10/yolov10n.pt')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, 'image', self.image_callback, 10)
        
    def image_callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = self.model(cv_image)
        annotated_frame = results[0].plot() 
        cv2.imshow('YOLOv10 Object Detection', annotated_frame)
        cv2.waitKey(25)
        
def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    rclpy.shutdown()