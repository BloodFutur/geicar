#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO  # To use YOLOv8


class PlateDetection(Node):
    def __init__(self):
        super().__init__('plate_detection')

        # CvBridge for the conversion between ROS and OpenCV
        self.bridge = CvBridge()

        # Load the YOLOv8 model for the plate detection
        self.model = YOLO("/root/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")  # Put the path of the AI model
        self.get_logger().info("YOLOv8 first model loaded successfully!")
        
        # Load the YOLOv8 model for the plate detection
        self.model = YOLO("/root/geicar/CharacterDetection/runs/detect/train3/weights/best.pt")  # Put the path of the AI model
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
        
        self.pub_compressed = self.create_publisher(
            CompressedImage,
            '/plate_detection/compressed',
            10
        )
        
        # Creation of a publisher to publish plate detection of images
        self.pub = self.create_publisher(
            Image,
            '/plate_detection',  # Topic for the annoted images
            10
        )

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
            annotated_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(annotated_image, dst_format='jpg')
            self.pub.publish(annotated_msg)
            self.pub_compressed.publish(annotated_msg_compressed)
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
