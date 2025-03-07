import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
import math


class OdomAndPathPublisher(Node):
    def __init__(self):
        super().__init__('sim_pose_path')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.path_pub = self.create_publisher(Path, 'tgt_path', 10)

        # Timer for periodic publishing
        self.timer_odom = self.create_timer(0.1, self.publish_odom)  # 10 Hz
        self.timer_path = self.create_timer(5.0, self.publish_path)  # Every 5 seconds

        # Variables for simulation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_velocity = 0.1  # Simulated forward velocity
        self.angular_velocity = 0.05  # Simulated rotational velocity
        self.path = self.generate_path()

        self.get_logger().info("Pose and Path Publisher Node Initialized")

    def publish_odom(self):
        """Publishes the current position of the robot."""
        # Update robot position
        self.robot_x += self.linear_velocity * math.cos(self.robot_theta) * 0.1
        self.robot_y += self.linear_velocity * math.sin(self.robot_theta) * 0.1
        self.robot_theta += self.angular_velocity * 0.1

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        odom_msg.pose.pose.position = Point(x=self.robot_x, y=self.robot_y, z=0.0)
        quat = self.euler_to_quaternion(0, 0, self.robot_theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom_msg)
        self.get_logger().debug(f"Published Odom: Position=({self.robot_x:.2f}, {self.robot_y:.2f}), Theta={self.robot_theta:.2f}")

    def publish_path(self):
        """Publishes a pre-defined path."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Add poses to the path message
        for pose in self.path:
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published Path with {len(self.path)} waypoints")

    def generate_path(self):
        """Generates a simple path with waypoints in a straight line."""
        path = []
        for i in range(20):  # Create 20 waypoints
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position = Point(x=i * 0.5, y=math.sin(i * 0.2), z=0.0)
            quat = self.euler_to_quaternion(0, 0, 0)
            pose.pose.orientation = Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3]
            )

            path.append(pose)
        return path

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Converts Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w


def main(args=None):
    rclpy.init(args=args)
    node = OdomAndPathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception as e:
        node.get_logger().error(f"Error in node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
