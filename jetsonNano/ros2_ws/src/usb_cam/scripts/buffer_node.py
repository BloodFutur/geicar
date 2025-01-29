#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from std_msgs.msg import String

class BufferingNode(Node):
    def __init__(self):
        super().__init__('BufferingNode')

        self.buffer_size = 10  # Max buffer size
        self.buffer = deque(maxlen=self.buffer_size)  # Circular buffer
        self.bridge = CvBridge()
        self.last_publish_time = time.time()

        # Subscribes to raw images
        self.sub = self.create_subscription(
            Image, 'raw_image_testing', self.image_callback, 10
        )
        #Subscribes to reset signal:
        self.signal_sub = self.create_subscription(
            String, 'Reset_signal', self.reset_callback, 10
        )

        # Publishes buffered images
        self.pub = self.create_publisher(Image, 'buffered_images', 10)
    def reset_callback(self, msg):
        if msg.data == "RESET":
            self.get_logger().info("Reset signal received. Resetting the buffer.")
            self.buffer.clear()

    def image_callback(self, msg):
        """Callback to buffer incoming images."""
        current_time = time.time()

        # Ensure images are buffered every 200ms
        if current_time - self.last_publish_time >= 0.2:
            self.buffer.append(msg)
            self.get_logger().info(f"Image added to buffer. Buffer size: {len(self.buffer)}")
            self.last_publish_time = current_time

            # Publish the buffered image
            self.pub.publish(msg)

        # Stop adding if buffer is full
        if len(self.buffer) == self.buffer_size:
            self.get_logger().info("Buffer is full. Stopping image buffering.")

def main(args=None):
    rclpy.init(args=args)
    node = BufferingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
