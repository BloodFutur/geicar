import pytest
import rclpy
from std_msgs.msg import Bool
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing
import time
from interfaces.msg import Ultrasonic
import unittest


# We indicate which node we want to test : in our case obstacles_detection_node
@pytest.mark.rostest
def generate_test_description():
    obstacles_detection_node = LaunchNode(
        package='detection',
        executable='obstacles_detection_node',
        name='obstacles_detection_node',
        output='screen',
    )
    return LaunchDescription([
        obstacles_detection_node,
        launch_testing.actions.ReadyToTest()

    ])


# Then we create a test node that will test obstacles_detection.
# It has to inherit from unittest.TestCase so that colcon test understands
class TestNode(unittest.TestCase):

    # We create the node and the variable to store the received value
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.test_node = rclpy.create_node('test_node')
        cls.received_data = []

    # Destruction of the node
    @classmethod
    def tearDownClass(cls):
        cls.test_node.destroy_node()
        rclpy.shutdown()

    # Here you can create the publishers and subscribers of the test node
    def setUp(self):
        self.publisher = self.test_node.create_publisher(Ultrasonic, 'us_data', 10)
        self.subscription = self.test_node.create_subscription(Bool, 'obstacles_detection', self.detected_message, 10)

    # Publishers/Subscribers destruction
    def tearDown(self):
        self.publisher.destroy()
        self.subscription.destroy()

    # Callback function of the subscriber
    def detected_message(self, msg):
        self.received_data.append(msg.data)

    # Beginning of the test
    # /!\This method needs to start by "test_" to be interpreted by colcon test/!\
    def test_node_behavior(self):
        '''
        We precise here the different inputs of the test
        We can make several one in one time to avoid redundancy
        We test several cases : above, below and equal to the detection limit
        Try to make user relevant test (like acceptance tests in the backlog)
        (no need to limit test like in an unit test)
        Ex :
        Given the vehicule is moving forward
        When an obstacle is detected
        Then the obstacle_detection_node needs to publish true messages
        '''
        test_cases = [
            {"front_left": 40, "front_right": 40, "expected_output": True},
            {"front_left": 80, "front_right": 80, "expected_output": False},
            {"front_left": 50, "front_right": 50, "expected_output": True},
            {"front_left": 60, "front_right": 60, "expected_output": False}
        ]
        # front_left, front_right and expected_output are not usable variables here
        for case in test_cases:
            with self.subTest(front_left=case["front_left"],
                              front_right=case["front_right"],
                              expected_output=case["expected_output"]
                              ):
                # You have to clear received_data to avoid interferences between tests, they are not independant
                self.received_data.clear()

                expected = case["expected_output"]
                input_msg = Ultrasonic()
                input_msg.front_left = case["front_left"]
                input_msg.front_right = case["front_right"]
                input_msg.front_center = 0
                input_msg.rear_center = 0
                input_msg.rear_left = 0
                input_msg.rear_right = 0

                # We wait for the node obstacles_detection to start (especially on my computer xD)
                # And to let the time for obstacles_detection node to stop publishing True/false data on the topic
                # So that u avoid interferences between test (especially relevant on my computer tbh)
                time.sleep(5)  # secondes

                # Then we publish the ultrasonic datas on /us_data
                self.publisher.publish(input_msg)

                # We wait for an answer of the obstacles_detection node
                timeout = 15.0  # secondes
                start_time = self.test_node.get_clock().now().nanoseconds / 1e9
                while not self.received_data:
                    rclpy.spin_once(self.test_node, timeout_sec=0.1)
                    if (self.test_node.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                        break

                # We check that the data has been received, if not we print the message in argument
                self.assertTrue(self.received_data, "Aucune donnée reçue sur /output_topic")
                # Then we check that it is the expected data
                self.assertEqual(
                    self.received_data[0],
                    case["expected_output"],
                    f"front left sensor : {input_msg.front_left}, front right sensor : {input_msg.front_right}, Expected : {expected}, Received : {self.received_data[0]}"
                )


if __name__ == '__main__':
    unittest.main()
