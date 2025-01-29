import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool
import math
import json

class GpsCalibrationNode(Node):
    def __init__(self):
        super().__init__('gps_calibration_node')

        # Subscriber to GPS topic
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10
        )
        
        # Service to start/stop calibration
        self.calibration_service = self.create_service(
            SetBool,
            'calibrate_gps',
            self.handle_calibration_request
        )

        # Internal variables
        self.calibrating = False
        self.gps_points = []

        self.get_logger().info('GPS Calibration Node started.')

    def gps_callback(self, msg):
        if self.calibrating:
            # Save GPS points when calibrating
            self.gps_points.append((msg.latitude, msg.longitude))
            self.get_logger().info(f'GPS Point added: {msg.latitude}, {msg.longitude}')

    def handle_calibration_request(self, request, response):
        if request.data:
            # Start calibration
            self.calibrating = True
            self.gps_points.clear()
            response.success = True
            response.message = 'Calibration started.'
            self.get_logger().info('Calibration started.')
        else:
            # Stop calibration
            self.calibrating = False
            if len(self.gps_points) > 1:
                orientation = self.compute_orientation() - math.pi / 2.0  # Facing east
                self.save_orientation_to_file(orientation)
                response.message = f'Calibration stopped. Orientation saved: {orientation}'
            else:
                response.message = 'Calibration stopped. Not enough points to compute orientation.'
            response.success = True
            self.get_logger().info(response.message)
        
        return response

    def compute_orientation(self):
        # Compute the orientation from the GPS points
        if len(self.gps_points) < 2:
            return 0.0

        sum_yaw = 0.0
        for i in range(len(self.gps_points) - 1):
            lat1, lon1 = self.gps_points[i]
            lat2, lon2 = self.gps_points[i + 1]
            delta_lon = lon2 - lon1
            x = math.sin(math.radians(delta_lon)) * math.cos(math.radians(lat2))
            y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - \
                math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(delta_lon))
            yaw = math.atan2(x, y)
            sum_yaw += yaw

        average_yaw = sum_yaw / (len(self.gps_points) - 1)
        return average_yaw

    def save_orientation_to_file(self, orientation):
        # Save the orientation to a file
        filename = 'orientation.json'
        with open(filename, 'w') as file:
            json.dump({'orientation': orientation}, file)
        self.get_logger().info(f'Orientation saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
