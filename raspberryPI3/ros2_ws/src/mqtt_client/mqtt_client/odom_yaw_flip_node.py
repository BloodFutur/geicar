import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) into a quaternion
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    
    return [qx, qy, qz, qw]

class OdomCorrectionNode(Node):
    def __init__(self):
        super().__init__('odom_correction_node')
        
        # Subscriber to the original odom topic
        self.subscription = self.create_subscription(
            Odometry,
            'odom/encoder',
            self.odom_callback,
            10)
        
        # Publisher for the corrected odom topic
        self.publisher = self.create_publisher(
            Odometry,
            'odom/encoder_flip',
            10)
        
        self.get_logger().info("Odom Correction Node has started.")

    def odom_callback(self, msg):
        # Extract the orientation quaternion from the odom message
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(*quaternion)
        
        # Flip the yaw to correct the orientation
        corrected_yaw = -yaw
        
        # Convert corrected Euler angles back to quaternion
        corrected_quaternion = quaternion_from_euler(roll, pitch, corrected_yaw)
        
        # Invert the Y position as well
        msg.pose.pose.position.y = -msg.pose.pose.position.y
        
        # Update the odometry message with the corrected orientation
        msg.pose.pose.orientation.x = corrected_quaternion[0]
        msg.pose.pose.orientation.y = corrected_quaternion[1]
        msg.pose.pose.orientation.z = corrected_quaternion[2]
        msg.pose.pose.orientation.w = corrected_quaternion[3]
        
        # Publish the corrected odometry
        self.publisher.publish(msg)
        self.get_logger().info("Published corrected odometry.")

def main(args=None):
    rclpy.init(args=args)
    node = OdomCorrectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
