import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions

class IMUPublisherNode(Node):
    def __init__(self, bag_path):
        super().__init__('imu_publisher_node')

        # Create a publisher to the /imu topic
        self.publisher = self.create_publisher(Imu, '/imu', 10)

        # Open the bag and read IMU messages
        self.bag_path = bag_path
        self.reader = SequentialReader()
        self.reader.open(
            StorageOptions(uri=self.bag_path, storage_id='sqlite3'),
            ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        )

        # Timer to publish messages at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_next_message)

    def publish_next_message(self):
        while self.reader.has_next():
            topic, serialized_data, timestamp = self.reader.read_next()
            
            # Filter for the /imu/data topic
            if topic == '/imu/data':
                # Deserialize the message
                imu_msg = deserialize_message(serialized_data, Imu)

                # Modify header.frame_id if needed
                imu_msg.header.frame_id = 'base_link'

                # Publish the message
                self.publisher.publish(imu_msg)
                self.get_logger().info(f'Published IMU message at timestamp: {timestamp}')
                return  # Exit after publishing to respect the timer interval

        # If no messages are left, stop the timer
        self.get_logger().info('Finished reading bag file.')
        self.destroy_timer(self.timer)


def main(args=None):
    rclpy.init(args=args)

    # Provide the path to your bag file
    bag_path = '/home/pi/geicar/raspberryPI3/ros2_ws/src/rosbag_data/db3/GPS_IMU.db3'
    imu_publisher_node = IMUPublisherNode(bag_path)

    rclpy.spin(imu_publisher_node)

    imu_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
