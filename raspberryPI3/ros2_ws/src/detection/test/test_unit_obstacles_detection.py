import pytest
import rclpy
from interfaces.msg import Ultrasonic
from std_msgs.msg import Bool
from detection.obstacles_detection_lib import detection
# We precise here the different inputs of the test
#(we can make several one in one time to avoid redundancy)
#We test several cases : above, below and equal to the detection limit    
@pytest.mark.parametrize('front_left_sensor_value, front_right_sensor_value, detection_distance, expected_output', [
    (100, 100, 50, False),
    (10, 100, 50, True),
    (100, 10, 50, True),
    (10, 10, 50, True),
    (50, 50, 50, True),
    (50, 100, 50, True),
    (100, 50, 50, True)
])

def test_detection(front_left_sensor_value:int, front_right_sensor_value:int, detection_distance:int, expected_output:Bool) :
    input_msg = Ultrasonic()
    input_msg.front_left = front_left_sensor_value
    input_msg.front_right = front_right_sensor_value
    input_msg.front_center = 0
    input_msg.rear_center = 0 
    input_msg.rear_left = 0 
    input_msg.rear_right = 0
    response = detection(input_msg, detection_distance)
    assert response == expected_output, f"Expected : {expected_output}, Received : {response}"
    
    