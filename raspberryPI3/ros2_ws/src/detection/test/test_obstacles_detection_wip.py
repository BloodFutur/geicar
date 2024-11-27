# import pytest
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool
# from launch import LaunchDescription
# from launch_ros.actions import Node as LaunchNode
# import launch_testing
# import time
# from interfaces.msg import Ultrasonic


# We indicate which node we want to test : in our case obstacle_detection
# @pytest.mark.launch_test
# def generate_test_description():
#     return LaunchDescription([
#         LaunchNode(
#             package='detection',
#             executable='obstacles_detection',
#             name='obstacles_detection',
#             output='screen'
#         ),
#         launch_testing.actions.ReadyToTest(),

#     ])
    
# We precise here the different inputs of the test
#(we can make several one in one time to avoid redundancy)
#We test several cases : above, below and equal to the detection limit    
# @pytest.mark.parametrize('front_left_sensor_value, front_right_sensor_value, expected_output', [
#     (100, 100, False),
#     (10, 100, True),
#     (100, 10, True),
#     (10, 10, True),
#     (50, 50, True),
#     (50, 100, True),
#     (100, 50, True)
# ])
# def test_obstacles_detection(front_left_sensor_value:int, front_right_sensor_value:int, expected_output:Bool):
    #rclpy.init()
    
    # We create an other test node that will communicate with the obstacles_detection node
    # test_node = rclpy.create_node('test_node')

    # received_output = []
    # received_output[0] = False

    # def detected_message(msg):
    #     received_output[0] = msg.data

    # The test node subs to the /obstacles_detection topic
    # subscription = test_node.create_subscription(Bool, 'obstacles_detection', detected_message, 10)

    # We publish sensor datas on the /us_data topic
    # publisher = test_node.create_publisher(Ultrasonic, 'us_data', 10)
    # input_msg = Ultrasonic()
    # input_msg.front_left = front_left_sensor_value
    # input_msg.front_right = front_right_sensor_value
    # input_msg.front_center = 0
    # input_msg.rear_center = 0 
    # input_msg.rear_left = 0 
    # input_msg.rear_right = 0 

    # publisher.publish(input_msg)

    # We create a timeout 
    #start_time = time.time()
    #timeout = 5.0  # seconds
    #while not received_output and time.time() - start_time < timeout:
    #    rclpy.spin_once(test_node, timeout_sec=0.1)

    # Result
    # assert received_output, "No received messages"
    # assert received_output[0] == expected_output, "Expected : {expected_output}, Received : {received_output[0]}"
    
    
    # End
    # test_node.destroy_node()
    # rclpy.shutdown()
