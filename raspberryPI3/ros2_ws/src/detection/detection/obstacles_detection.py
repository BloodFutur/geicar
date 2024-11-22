import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from interfaces.msg import Ultrasonic


class ObstaclesDetection(Node):
    """
    Node to detect obstacles using ultrasonic sensors.
    Publishes a Bool message to 'obstacles_detection' topic when an obstacle is detected.
    """
    def __init__(self):
        super().__init__('obstacles_detection')
        self.publisher = self.create_publisher(Bool, 'obstacles_detection', 10)
        self.get_logger().info('obstacles_detection_node started')
        self.subscriber = self.create_subscription(Ultrasonic, 'us_data', self.obstacle_detected, 10)
        # Detection treshold parameter in centimeters
        self.detection_distance = self.declare_parameter('detection_distance', 50).get_parameter_value().integer_value

    def obstacle_detected(self, msg_us: Ultrasonic):
        # We check the two sensors each because they don't look at the same place
        # ex : if obstacle only at the left side, the right sensor won't see it
        # We only send a true msg if we detect an obstacle so we don't need a false case
        # We do that so it's easier for the future subscribers of the /obstacles_detection subscribers to process the msg
        # And bc we don't need to change anything in the behavior of the car if there is no obstacles
        if msg_us.front_left <= self.detection_distance or msg_us.front_right <= self.detection_distance:
            self.get_logger().debug(f'Obstacle detected at : right : {msg_us.front_right} left :  {msg_us.front_left} ')
            response = Bool()
            response.data = True
            self.publisher.publish(response)


def main(args=None):
    rclpy.init(args=args)

    obstacles_detection_node = ObstaclesDetection()

    rclpy.spin(obstacles_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacles_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
