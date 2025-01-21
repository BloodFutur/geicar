#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ultralytics import YOLO  # YOLOv8 for detection
from Tilt_Corrector import Rotator  # Import Tilt Correction
from Segmentor import CharacterSegmentation  # Import Character Segmentation
import easyocr  # For OCR

class PlateDetection(Node):
    def __init__(self):
        super().__init__('LPDPipelineNode')

        # CvBridge for the conversion between ROS and OpenCV
        self.bridge = CvBridge()

        # Load the YOLOv8 model
        self.model = YOLO("/root/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")
        self.get_logger().info("YOLOv8 model loaded successfully!")

        # Initialize EasyOCR
        self.reader = easyocr.Reader(['en'])
        self.get_logger().info("EasyOCR loaded and initialized successfully!")

        # Initilize local variables
        self.buffer_size = 10 # Buffer size for detected texts
        self.detected_texts = [] # Buffer for detected texts

        # Create a Quality of Service (QoS) profile for the subscriber
        """
        The QoSProfile configuration defines the behavior of message delivery and storage for the topic subscription. 
        By setting the reliability to RELIABLE, it ensures that all messages are delivered to the subscriber even if the subscriber is busy.
        This makes sure that the License Plate Detection Pipeline proccesses the buffered images in order.
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Ensure all messages are reliably delivered to the subscriber.
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Queue size
            )
        # Subscribe to the buffer topic
        self.sub = self.create_subscription(
            Image,
            'buffered_images',
            self.image_callback,
            qos_profile
        )

        # Publishers
        self.pub_text = self.create_publisher(String, '/detected_text', 10)  

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        """
        License Plate Detection Pipeline: 
        Detection of the Region of Interest ( ROI) using Yolov8n constum model
        Cropping the frame using the Bounding Box (LP_BB) to contain only the ROI
        Tilt correction in case of detected tilt to enhance character segmentation
        Character Segmentation using openCV.
        Reading the characters using EasyOCR 
        Parameters: 
        Input: Frame
        Output: Detected Text 
        """
        # Step 1: Detection and extraction of the ROI
        results = self.model(frame)
        if len(results) > 0 and len(results[0].boxes) > 0:
            box = results[0].boxes[0]  # First bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates

            # Crop license plate from the image
            cropped_plate = frame[y1:y2, x1:x2]
            self.get_logger().info(f"License plate detected and cropped.")
        

            # Step 2: Tilt correction
            LP_BB = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
            rotator = Rotator(LP_BB, cropped_plate)
            rotated_plate = rotator.rotate_image()

            # Step 3: Character segmentation
            segmentor = CharacterSegmentation()
            char_list = segmentor.segment_characters(rotated_plate)

            # Step 4: Text Recognition
            extracted_text = ""
            for i, char_img in enumerate(char_list):
                ocr_result = self.reader.readtext(char_img, detail=0, allowlist="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
                if ocr_result:
                    extracted_text += ocr_result[0]  
            
            # Step 5: Add extracted text to buffer
                self.detected_texts.append(extracted_text)
                self.get_logger().info(f"Extracted Text: {extracted_text}")

        else:
            self.get_logger().info("No license plate detected.")
        # Publish all 10 detected texts once buffer is full
        if len(self.detected_texts) == self.buffer_size:
            for text in self.detected_texts:
                self.pub.publish(String(data=text))
            self.get_logger().info("Published 10 detected texts.")
            self.detected_texts.clear()


def main(args=None):
    rclpy.init(args=args)
    plate_detection_node = PlateDetection()

    try:
        rclpy.spin(plate_detection_node)
    except KeyboardInterrupt:
        pass

    plate_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
