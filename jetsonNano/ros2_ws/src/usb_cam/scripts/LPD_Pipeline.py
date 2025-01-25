#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ultralytics import YOLO  # YOLOv8 for detection
from Tilt_Corrector import Rotator  # Import Tilt Correction
from Segmentor import CharacterSegmentation  # Import Character Segmentation
import easyocr  # For OCR
import torch
import re
from collections import defaultdict


class PlateDetection(Node):
    def __init__(self):
        super().__init__('LPDPipelineNode')

        # CvBridge for the conversion between ROS and OpenCV
        self.bridge = CvBridge()
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # Load the YOLOv8 model
        self.model = YOLO("/root/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")
        # Move the model to the GPU
        self.model.to(device)

        #self.model=YOLO('/home/pi/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt')
        self.get_logger().info("YOLOv8 model loaded successfully!")

        # Initialize EasyOCR
        self.reader = easyocr.Reader(['en'],gpu=True)
        self.get_logger().info("EasyOCR loaded and initialized successfully!")

        # Initilize local variables
        self.buffer = []
        self.max_buffer_size = 10
        self.frame_skip = 5  # Sample every 5th frame
        self.frame_count = 0
        self.latitude = 0.0
        self.longitude = 0.0
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
        # Subscription to gps topic
        self.sub2 = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # Topic of the gps  
            self.gps_callback,
            10
        )
        # Publishers
        self.pub_text = self.create_publisher(String, '/verified_text', 10) 
        self.timer = self.create_timer(15.0, self.force_process_images)

    #Timeout check     
    def force_process_images(self):
        if len(self.buffer) > 0:
            self.get_logger().info("Processing images due to timeout.")
            self.process_images()
            self.buffer = []     
    def gps_callback(self, msg):
        # Extract latitude and longitude from NavSatFix message
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.get_logger().info(f"Received GPS coordinates: {self.latitude}, {self.longitude}")
    
    
    def image_callback(self, image_msg):
        self.get_logger().info("Ready to process images")
        """Handles incoming images, buffers, and triggers processing."""
        self.frame_count += 1

        detected_texts = [] #Buffer for detected texts to verify 
        confidences=[] # Buffer for detected texts confidences 
        # Skip frames to sample every N frames
        if self.frame_count % self.frame_skip != 0:
            return
        
        try:
        # Convert ROS image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Add the image to the buffer
        self.buffer.append(frame)
        self.get_logger().info(f"Buffered image {len(self.buffer)}/{self.max_buffer_size}")
        # Process the added image 
        detected_text,confidence=self.process_images(frame)
        detected_texts.append(detected_text)
        confidences.append(confidence)
        # Verify text: 
        verified_text=self.verify_text(detected_texts,confidences)
        # Publish Results
        self.get_logger().info("Plate: {verified_text}, Latitude: {self.latitude}, Longitude: {self.longitude}")
        msg = String()
        msg.data = f"Plate: {verified_text}, Latitude: {self.latitude}, Longitude: {self.longitude}"
        self.pub_text.publish(msg)
        # If buffer is full, clear the buffer 
        if len(self.buffer) == self.max_buffer_size:
            self.buffer = []  # Clear the buffer for the next batch
            
    def process_images(self,image):

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

        # Initialization of the list for confidnece scores:
        confidence=[]
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
            self.get_logger().info("Tilt corrected")
            # Step 3: Character segmentation
            segmentor = CharacterSegmentation()
            char_list = segmentor.segment_characters(rotated_plate)
            self.get_logger().info("Segmentation done")
            # Step 4: Text Recognition
            extracted_text = ""
            for char_img in char_list:
                ocr_result = self.reader.readtext(char_img, detail=1, allowlist="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
                if ocr_result:
                    extracted_text += ocr_result[1]  
                    confidence.append(ocr_result[2])
            self.get_logger().info(f"Extracted Text: {extracted_text}")
        else:
            self.get_logger().info("No license plate detected.")
            extracted_text="0000000"
            confidence=[0,0,0,0,0,0,0]
        return extracted_text,confidence    
            
    def validate_plate(self,plate):
        # Regex for current French license plate format (AA123AA)
        plate_format = re.compile(r"^[A-Z]{2}\d{3}[A-Z]{2}$")
        return bool(plate_format.match(plate))
    
    def verify_text(self,detected_texts, confidences):
        # Step 1: Pre-process detected texts 
        preprocessed_texts = []
        for text in detected_texts:
            # Remove all characters except ABCDEFGHIJKLMNOPQRSTUVWXYZ123456789
            cleaned_text = re.sub(r"[^A-Z0-9]", "", text.upper())
            preprocessed_texts.append(cleaned_text)
    
        # Step 2: Extract valid substrings of license plates from the detected text length 7 
        valid_substrings = []
        for text in preprocessed_texts:
            # Look for all substrings of length 7 matching the valid format
            matches = re.findall(r"[A-Z]{2}\d{3}[A-Z]{2}", text)
            valid_substrings.extend(matches)
    
        # Step 3: Initialize weighted scores for each position
        plate_length = 7
        weighted_scores = [defaultdict(float) for _ in range(plate_length)]
    
        # Step 4: Apply weighted voting for valid substrings
        for text, conf in zip(valid_substrings, confidences):
            for i, char in enumerate(text):
                if i in [0, 1, 5, 6]:  # Positions of letters in the license plate 
                    if char.isalpha():
                        weighted_scores[i][char] += conf[i]
                elif i in [2, 3, 4]:  # Positions of numbers in the license plate
                    if char.isdigit():
                        weighted_scores[i][char] += conf[i]
    
        # Step 5: Construct the verified plate from weighted scores
        verified_text = ""
        for char_scores in weighted_scores:
            if char_scores:
                verified_text += max(char_scores, key=char_scores.get)  # Select character with highest score
            else:
                verified_text += "_"  # Placeholder for missin<g positions
    
        # Step 6: Validate the final text
        if self.validate_plate(verified_text):
            return verified_text
        else:
            return "INVALID"




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
