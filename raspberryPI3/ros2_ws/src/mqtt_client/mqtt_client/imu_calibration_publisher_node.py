import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import json
import math
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')

        # Publisher for IMU messages
        self.imu_publisher = self.create_publisher(Imu, 'imu/gps_cal', 10)
        
        # Publisher for Odometry messages
        self.odom_publisher = self.create_publisher(Odometry, 'odom/gps_cal', 10)

        # Load the orientation from the file
        self.orientation = self.load_orientation_from_file()

        # Publish the IMU message periodically
        self.timer = self.create_timer(1.0, self.publish_imu_message)
        self.get_logger().info('IMU Publisher Node started.')

    def load_orientation_from_file(self):
        try:
            with open('orientation.json', 'r') as file:
                data = json.load(file)
                yaw = data['orientation']
                return self.yaw_to_quaternion(yaw)
        except FileNotFoundError:
            self.get_logger().error('orientation.json file not found. Using default orientation.')
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    def yaw_to_quaternion(self, yaw):
        """ Convert yaw angle to quaternion """
        half_yaw = yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def publish_imu_message(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation = self.orientation

        # Assuming no angular velocity or linear acceleration measurements
        imu_msg.angular_velocity_covariance[0] = -1  # Indicates no data
        imu_msg.linear_acceleration_covariance[0] = -1  # Indicates no data

        self.imu_publisher.publish(imu_msg)
        self.get_logger().info('IMU message published with orientation: %s' % str(self.orientation))

        # publish odom message with pose 0,0,0 and orientation self.orientation
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.orientation = self.orientation
        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
