#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from easyocr import Reader

# Extern scripts import for the pipeline
from image_crop import crop_images_for_detection
from Image_Preprocessing import preprocess_image


class PlateDetectionPipeline(Node):
    def __init__(self):
        super().__init__('plate_detection_pipeline')

        self.bridge = CvBridge()
        self.yolo_model = YOLO("/root/geicar/PlaqueDetection/runs/detect/train7/weights/best.pt")
        self.ocr_reader = Reader(['en'], gpu=True) 

        self.get_logger().info("YOLO model and EasyOCR initialized successfully!")

        # Subscribe to the camera topic
        self.sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )

        # Publisher for processed images
        self.pub = self.create_publisher(
            Image,
            '/plate_detection_pipeline',
            10
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # YOLO detection
        results = self.yolo_model(frame)
        cropped_images = []

        for result in results:
            for box in result.boxes:
                # Extract bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                # Crop the image using bounding box coordinates
                cropped_image = frame[y1:y2, x1:x2]
                cropped_images.append(cropped_image)

        # Save cropped images to a directory (optional)
        output_dir = "./cropped_images"
        for idx, img in enumerate(cropped_images):
            output_path = f"{output_dir}/plate_{idx}.png"
            cv2.imwrite(output_path, img)

        recognized_texts = []
        for cropped_image in cropped_images:
            # Preprocess each cropped image
            preprocessed_image = preprocess_image(cropped_image)

            # OCR inference
            text_results = self.ocr_reader.readtext(preprocessed_image)
            recognized_texts.append(text_results)

        # Draw detections and OCR results
        annotated_frame = frame.copy()
        for result, texts in zip(results, recognized_texts):
            for box, text in zip(result.boxes, texts):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if text:
                    cv2.putText(annotated_frame, text[1], (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish annotated frame
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.pub.publish(annotated_msg)
            self.get_logger().info("Published annotated image with OCR results.")
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PlateDetectionPipeline()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
