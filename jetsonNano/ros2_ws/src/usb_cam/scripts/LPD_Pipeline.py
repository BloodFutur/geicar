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
        #self.model=YOLO('/home/pi/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt')
        self.get_logger().info("YOLOv8 model loaded successfully!")

        # Initialize EasyOCR
        self.reader = easyocr.Reader(['en'])
        self.get_logger().info("EasyOCR loaded and initialized successfully!")

        # Initilize local variables
        self.buffer = []
        self.max_buffer_size = 10
        self.frame_skip = 2  # Sample every 2rd frame
        self.frame_count = 0
        # Create a Quality of Service (QoS) profile for the subscriber
        """
        The QoSProfile configuration defines the behavior of message delivery and storage for the topic subscription. 
        By setting the reliability to RELIABLE, it ensures that all messages are delivered to the subscriber even if the subscriber is busy.
        This makes sure that the License Plate Detection Pipeline proccesses the buffered images in order.
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Ensure all messages are reliably delivered to the subscriber.
            history=HistoryPolicy.KEEP_LAST,
            depth=20  # Queue size
            )
        # Subscribe to the buffer topic
        self.sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            100
        )

        # Publishers
        self.pub_text = self.create_publisher(String, '/verified_text', 10) 
        self.timer = self.create_timer(15.0, self.force_process_images)
    def force_process_images(self):
        if len(self.buffer) > 0:
            self.get_logger().info("Processing images due to timeout.")
            self.process_images()
            self.buffer = []     
    def image_callback(self, image_msg):
        self.get_logger().info("Ready to process images")
        """Handles incoming images, buffers, and triggers processing."""
        self.frame_count += 1

        # Skip frames to sample every N frames
        if self.frame_count % self.frame_skip != 0:
            return

        # Convert ROS Image to OpenCV format
        try:
        # Convert ROS image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Add the image to the buffer
        self.buffer.append(frame)
        self.get_logger().info(f"Buffered image {len(self.buffer)}/{self.max_buffer_size}")

        # If buffer is full, process the images
        if len(self.buffer) == self.max_buffer_size:
            self.process_images()
            self.buffer = []  # Clear the buffer for the next batch
            
    def process_images(self):

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
        detected_texts = []
        for image in self.buffer:
            # Step 1: Detection and extraction of the ROI
            results = self.model(image)
            if len(results) > 0 and len(results[0].boxes) > 0:
                box = results[0].boxes[0]  # First bounding box
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates

                # Crop license plate from the image
                cropped_plate = image[y1:y2, x1:x2]
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
                for char_img in char_list:
                    ocr_result = self.reader.readtext(char_img, detail=0, allowlist="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
                    if ocr_result:
                        extracted_text += ocr_result[0]  
            
                # Step 5: Add extracted text to buffer
                    #self.detected_texts.append(extracted_text)
                    self.get_logger().info(f"Extracted Text: {extracted_text}")
                    detected_texts.append(extracted_text)
                
            else:
                self.get_logger().info("No license plate detected.")
                detected_texts.append("0000000")
        # Verify the text
        verified_text=""
        max_len = max(len(text) for text in detected_texts)
        # Iterate over each character position
        for char_pos in range(max_len):
            char_count = {}
            # Count occurrences of each character at the current position
            for text in detected_texts:
                if char_pos < len(text):
                    char = text[char_pos]
                    char_count[char] = char_count.get(char, 0) + 1

            # Validate character appearing in at least 70% of texts in the same position
            threshold = len(detected_texts) * 0.7
            for char, count in char_count.items():
                if count >= threshold:
                    verified_text += char
                    break
        # Publish Verified text
        msg = String()
        msg.data = verified_text
        self.pub_text.publish(msg)
        

            
        

def main(args=None):
    rclpy.init(args=args)
    LPDPipelineNode = PlateDetection()

    try:
        rclpy.spin(LPDPipelineNode)
    except KeyboardInterrupt:
        pass

    LPDPipelineNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
