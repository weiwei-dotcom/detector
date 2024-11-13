import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import random
import yaml
import cv2

class YoloDetector(Node):
    """
    Class to detect objects using YOLO.
    """
    def __init__(self):
        with open('/home/wl/Documents/detector/src/camera_node/config/config.yaml', 'r', encoding='utf-8') as file:
            self.config = yaml.safe_load(file)
        super().__init__('yolo_detector')
        self.model = YOLO(self.config['model_path'])

        self.bridge = CvBridge()
        self.colors = [[]]
        # 随机产生颜色数组
        for color_index in range(1000):
            self.colors.append([random.randint(0,255), random.randint(0,255), random.randint(0,255)])
        self.image_subscriber = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.get_logger().info("yolo_node launch success ! ! !")
        
    def image_callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = self.model.predict(cv_image, conf=self.config['confidence'])
        for result in results:
            for box in result.boxes:
                cv2.rectangle(cv_image, 
                              (int(box.xyxy[0][0]), int(box.xyxy[0][1])), 
                              (int(box.xyxy[0][2]), int(box.xyxy[0][3])), 
                              (self.colors[int(box.cls[0]+1)][0],self.colors[int(box.cls[0]+1)][1],self.colors[int(box.cls[0]+1)][2]), 
                              self.config['box_thickness'])
                cv2.putText(cv_image,
                            f"{result.names[int(box.cls[0])]}",
                            (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                            cv2.FONT_HERSHEY_PLAIN,
                            self.config['font_size'],
                            (self.colors[int(box.cls[0]+1)][0],self.colors[int(box.cls[0]+1)][1],self.colors[int(box.cls[0]+1)][2]),
                            self.config['text_thickness'])
                cv2.putText(cv_image,
                            f"({int(box.xyxy[0][0])}, {int(box.xyxy[0][1])})",
                            (int(box.xyxy[0][0]), int(box.xyxy[0][1]) + 22),
                            cv2.FONT_HERSHEY_PLAIN,
                            self.config['font_size'],
                            (self.colors[int(box.cls[0]+1)][0],self.colors[int(box.cls[0]+1)][1],self.colors[int(box.cls[0]+1)][2]),
                            self.config['text_thickness'])
        cv2.imshow('YOLOv10 Object Detection', cv_image)
        cv2.waitKey(self.config['camera_fps'])
        
def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()

    rclpy.spin(yolo_detector)
    rclpy.shutdown()