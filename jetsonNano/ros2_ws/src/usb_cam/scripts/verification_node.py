#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VerificationNode(Node):
    def __init__(self):
        super().__init__('Extracted Text Verification Node')

        self.buffer_size = 10 # Buffer size for detected texts
        self.detected_texts = []  # Buffer for detected texts

        # Subscribes to detected texts
        self.sub = self.create_subscription(
            String, 'detected_texts', self.text_callback, 10
        )

        # Publish verified text
        self.pub = self.create_publisher(String, 'verified_text', 10)

    def text_callback(self, msg):
        """Buffer detected texts and validate when full."""
        self.detected_texts.append(msg.data)
        self.get_logger().info(f"Received extracted text: {msg.data}")

        # Process when buffer is full
        if len(self.detected_texts) == self.buffer_size:
            verified_text = self.validate_text()
            self.publish_verified_text(verified_text)
            self.detected_texts.clear()

    def validate_text(self):
        """Validate detected text across buffered images."""
        if not self.detected_texts:
            return ""

        # Character-wise validation
        verified_text = ""
        # In case of extracted characters in the blue region of the license plate, we define max_len as the maximum length of texts in the buffer
        max_len = max(len(text) for text in self.detected_texts)
        # Iterate over each character position
        for char_pos in range(max_len):
            char_count = {}
            # Count occurrences of each character at the current position
            for text in self.detected_texts:
                if char_pos < len(text):
                    char = text[char_pos]
                    char_count[char] = char_count.get(char, 0) + 1

            # Validate character appearing in at least 70% of texts in the same position
            threshold = len(self.detected_texts) * 0.7
            for char, count in char_count.items():
                if count >= threshold:
                    verified_text += char
                    break

        return verified_text

    def publish_verified_text(self, text):
        """Publish the verified text."""
        self.pub.publish(String(data=text))
        self.get_logger().info(f"Verified Text: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = VerificationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
