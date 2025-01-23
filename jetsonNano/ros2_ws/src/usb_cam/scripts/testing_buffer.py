#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class TestingBufferNode(Node):
    def __init__(self, image_folder):
        super().__init__('TestingBufferNode')

        self.image_folder = image_folder  # Folder containing the images
        self.image_paths = self.load_images(image_folder)  # List of image paths
        self.bridge = CvBridge()
        self.current_index = 0  # Index to track which image to publish

        # Publisher for the "raw_image" topic
        self.publisher = self.create_publisher(Image, 'raw_image_testing', 10)

        # Timer to publish images every 100ms
        self.timer = self.create_timer(0.1, self.publish_image)  # 100ms interval

    def load_images(self, folder):
        """Load all image paths from a given folder."""
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']  # Supported extensions
        return [
            os.path.join(folder, f)
            for f in sorted(os.listdir(folder))
            if os.path.splitext(f)[1].lower() in image_extensions
        ]

    def publish_image(self):
        """Publishes the next image in the list."""
        if not self.image_paths:
            self.get_logger().error(f"No images found in the folder: {self.image_folder}")
            return

        # Load the current image
        image_path = self.image_paths[self.current_index]
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            return

        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Publish the image
        self.publisher.publish(ros_image)
        self.get_logger().info(f"Published image: {image_path}")

        # Move to the next image in the list (looping back to the start if needed)
        self.current_index = (self.current_index + 1) % len(self.image_paths)


def main(args=None):
    rclpy.init(args=args)

    # Specify the folder containing the images
    image_folder = '/home/pi/jetsonNano/ros2_ws/src/usb_cam/scripts/testing_images'  # Replace with your actual image folder

    node = TestingBufferNode(image_folder=image_folder)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
