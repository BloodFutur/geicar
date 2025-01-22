#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from cv_bridge import CvBridge
from ultralytics import YOLO  # To use YOLOv8
from std_msgs.msg import String


class PlateDetection(Node):
    def __init__(self):
        super().__init__('plate_detection')

        # CvBridge for the conversion between ROS and OpenCV
        self.bridge = CvBridge()

        # Load the YOLOv8 model for the plate detection
        # self.model = YOLO("/home/pi/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")  # Put the path of the AI model
        self.model = YOLO("/root/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")  # Put the path of the AI model
        self.get_logger().info("YOLOv8 first model loaded successfully!")
        
        # Load the YOLOv8 model for the plate detection
        self.model2 = YOLO("/root/geicar/CharacterDetection/runs/detect/train3/weights/best.pt")  # Put the path of the AI model
        # self.model2 = YOLO("/home/pi/geicar/CharacterDetection/runs/detect/train3/weights/best.pt")  # Put the path of the AI model
        self.get_logger().info("YOLOv8 second model loaded successfully!")

        # Subscribe to the camera topic
        self.sub = self.create_subscription(
            Image,
            'image_raw',  # Topic of the camera 
            self.image_callback,
            10
        )
        
        self.sub2 = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # Topic of the gps  
            self.gps_callback,
            10
        )

        # Creation of a publisher to publish plate detection of images
        self.pub_plate = self.create_publisher(
            Image,
            '/plate_detection',  # Topic for the annoted images
            10
        )
        
        self.pub_plate_compressed = self.create_publisher(
            CompressedImage,
            '/plate_detection/compressed',
            10
        )
        
        # Creation of a publisher to publish plate detection of images
        self.pub_char = self.create_publisher(
            Image,
            '/character_detection',  # Topic for the annoted images
            10
        )

        self.pub_char_compressed = self.create_publisher(
            CompressedImage,
            '/character_detection/compressed',
            10
        )
        
        self.pub_plate_text = self.create_publisher(
            String,
            '/detected_plate_text',
            10
        )
        
        self.latitude = 0.0
        self.longitude = 0.0
        
    def gps_callback(self, msg):
        # Extract latitude and longitude from NavSatFix message
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.get_logger().info(f"Received GPS coordinates: {self.latitude}, {self.longitude}")
    
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
                # Extracte the coordinate of the box and every informations associated
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Rectangle coordinate
                confidence = box.conf[0]  # Level of confidence
                class_id = box.cls[0]  # Detected Class
                label = f"{self.model.names[int(class_id)]}: {confidence:.2f}"

                # Add margin around the detected plate (5% of box size)
                margin_x = int(0.05 * (x2 - x1))
                margin_y = int(0.05 * (y2 - y1))
                x1 = max(0, x1 - margin_x)
                y1 = max(0, y1 - margin_y)
                x2 = min(frame.shape[1], x2 + margin_x)
                y2 = min(frame.shape[0], y2 + margin_y)
                
                # Draw the bow and label of the annoted image
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Extract plate region
                plate_roi = frame[y1:y2, x1:x2]

                # Character detection on the extracted plate
                char_results = self.model2(plate_roi)
                characters = []    
                
                for char_result in char_results:
                    for char_box in char_result.boxes:
                        cx1, cy1, cx2, cy2 = map(int, char_box.xyxy[0])
                        char_conf = char_box.conf[0]
                        char_id = char_box.cls[0]
                        char_label = self.model2.names[int(char_id)]  # only keep the character
                        characters.append((cx1, char_label))  # put the character with x coordinate
                        
                        # Draw character bounding box (adjusted to the original image coordinates)
                        cv2.rectangle(plate_roi, (x1 + cx1, y1 + cy1), (x1 + cx2, y1 + cy2), (255, 0, 0), 1)
                        cv2.putText(plate_roi, char_label, (x1 + cx1, y1 + cy1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                        
                characters.sort(key=lambda x: x[0])
                plate_text = ''.join([char[1] for char in characters])
                self.get_logger().info(f"Detected Plate: {plate_text}")
                
                try:
                    plate_msg = String()
                    plate_msg.data = f"Plate: {plate_text}, Latitude: {self.latitude}, Longitude: {self.longitude}"
                    self.pub_plate_text.publish(plate_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish plate text: {e}")
                    
                # try:
                #     #Publish the cropped and annotated plate image
                #     annotated_msg_characters = self.bridge.cv2_to_imgmsg(plate_roi, encoding="bgr8")
                #     annotated_msg_compressed_characters = self.bridge.cv2_to_compressed_imgmsg(plate_roi, dst_format='jpg')
                #     self.pub_char.publish(annotated_msg_characters)
                #     self.pub_char_compressed.publish(annotated_msg_compressed_characters)
                #     self.get_logger().info("Published annotated image on /character_detection")
                # except Exception as e:
                #     self.get_logger().error(f"Failed to publish character detection image: {e}")

        # Publish annoted image in a ROS topic
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            annotated_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(annotated_image, dst_format='jpg')
            self.pub_plate.publish(annotated_msg)
            self.pub_plate_compressed.publish(annotated_msg_compressed)
            self.get_logger().info("Published annotated image on /plate_detection")
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")


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
