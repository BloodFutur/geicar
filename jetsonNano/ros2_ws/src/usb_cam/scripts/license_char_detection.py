#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO  # To use YOLOv8
import pytesseract # add to the READ_ME file
import logging

# Configuration du module logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

class CharDetection(Node):
    def __init__(self):
        super().__init__('license_char_detection')

        # CvBridge for the conversion between ROS and OpenCV
        self.bridge = CvBridge()

        # Load the YOLOv8 model
        self.model = YOLO("/root/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")  # Put the path of the AI model
        #self.model = YOLO("/home/pi/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")
        self.get_logger().info("YOLOv8 model loaded successfully!")

        # Subscribe to the camera topic
        self.sub = self.create_subscription(
            Image,
            'image_raw',  # Topic of the camera 
            self.image_callback,
            10
        )

        # Creation of a publisher to publish plate detection of images
        self.pub = self.create_publisher(
            Image,
            '/plate_detection',  # Topic for the annoted images
            10
        )
        
    def validate_plate_format(self, plate_text):
        """
        Validates and adjusts the plate format to 'XX-XXX-XX' if necessary.
        """
        plate_text = plate_text.replace(" ", "")  # Remove spaces

        # Check if it matches the desired format 'XX-XXX-XX'
        if re.match(r'^[A-Z0-9]{2}-[A-Z0-9]{3}-[A-Z0-9]{2}$', plate_text):
            return plate_text  # Plate is already valid

        # If the detected plate contains excessive characters or incorrect format
        plate_parts = plate_text.split('-')
        if len(plate_parts) == 3:  # Plaque déjà segmentée
            if len(plate_parts[0]) > 2:
                plate_parts[0] = plate_parts[0][-2:]  # Prendre les deux derniers caractères
            if len(plate_parts[1]) > 3:
                plate_parts[1] = plate_parts[1][:3]  # Garder les trois premiers caractères
            if len(plate_parts[2]) > 2:
                plate_parts[2] = plate_parts[2][:2]  # Garder les deux premiers caractères
            return '-'.join(plate_parts)

        # If no valid format is possible, return an empty string
        return ""

    def image_callback(self, msg):
        try:
            # Convertion of the ROS message as an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Applied YOLOv8 on the image
        results = self.model(frame)

        # Annoted the image with the results
        annotated_image = frame.copy()

        for result in results:
            for box in result.boxes:
                # Extract the coordinate of the box and every informations associated
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Rectangle coordinate
                confidence = box.conf[0]  # Level of confidence
                class_id = box.cls[0]  # Detected Class
                label = f"{self.model.names[int(class_id)]}: {confidence:.2f}"

                plate_roi = frame[y1:y2, x1:x2] # Keep only the part of the image with the plate
                
                try:
                    # Preprocessing for OCR
                    gray_plate = cv2.cvtColor(plate_roi, cv2.COLOR_BGR2GRAY)
                    gray_plate = cv2.equalizeHist(gray_plate)
                    _, threshold_plate = cv2.threshold(gray_plate, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

                    # Use Tesseract OCR
                    config = '-c tessedit_char_whitelist=ABCDEFGHJKLMNPQRSTVWXYZ0123456789 --psm 3'
                    plate_text = pytesseract.image_to_string(threshold_plate, config=config).strip()
                    #plate_text = pytesseract.image_to_string(threshold_plate, config='--psm 8').strip()

                    plate_text = self.validate_plate_format(plate_text)
                    
                    if plate_text:
                        # Annoting the image with the text that has been detected
                        cv2.putText(annotated_image, plate_text, (x1, y2 + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        logger.info(f"Detected Plate Text: {plate_text}")
                    else:
                        logger.warning("Invalid plate format detected. Skipping annotation.")

                except Exception as e:
                    # Log of errors without stoping the program
                    logger.error(f"Failed to process plate text: {e}")
                    
                    
                # Draw the bow and label of the annoted image
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show annoted image on OpenCV
        #cv2.imshow("Plate Detection", annotated_image)
        #cv2.waitKey(5)

        # Publish annoted image in a ROS topic
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            self.pub.publish(annotated_msg)
            self.get_logger().info("Published annotated image on /plate_detection")
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")


def main(args=None):
    rclpy.init(args=args)

    license_char_detection_node = CharDetection()

    try:
        rclpy.spin(license_char_detection_node)
    except KeyboardInterrupt:
        pass

    license_char_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
