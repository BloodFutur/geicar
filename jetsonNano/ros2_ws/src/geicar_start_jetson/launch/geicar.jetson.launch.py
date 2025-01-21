from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    lidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        emulate_tty=True
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        emulate_tty=True
    )

    system_check_ack_node = Node(
        package="system_check_ack",
        executable="system_check_ack_node",
        emulate_tty=True
    )
    
    # Plate detection node
    plate_detection_node = Node(
        package='usb_cam',  # name of the package where you can find the .py script of the plate detection
        executable='plate_detection.py',  # Name of the Python file
        name='plate_detection',  # Name of the ROS node
        output='screen',
        parameters=[{
            'camera_topic': '/image_raw',  # If needed you can change the name of the camera topic
            'detection_threshold': 0.5,  
        }],
        emulate_tty=True
    )

    # Buffering  node
    BufferingNode= Node(
        package='usb_cam',  # name of the package where you can find the .py script of the buffering
        executable='buffer_node.py',  # Name of the Python file
        name='BufferingNode',  # Name of the ROS node
        output='screen',
        parameters=[{
            'camera_topic': '/image_raw',  # If needed you can change the name of the camera topic 
        }],
        emulate_tty=True
    )
    # LPD Pipeline Node
    PlateDetection= Node(
        package='usb_cam',  # name of the package where you can find the .py script of the buffering
        executable='LPD_Pipeline.py',  # Name of the Python file
        name='LPDPipelineNode',  # Name of the ROS node
        output='screen',
        parameters=[{
            'buffer_topic': '/buffered_images',  # Buffered images topic
        }],
        emulate_tty=True
    )
    # Verification Node
    VerificationNode= Node(
        package='usb_cam',  # name of the package where you can find the .py script of the buffering
        executable='verification_node.py',  # Name of the Python file
        name='VerificationNode',  # Name of the ROS node
        output='screen',
        parameters=[{
            'Buffer_Extracted_text_topic': '/detected_texts',  # Buffered images topic
        }],
        emulate_tty=True
    )


    ld.add_action(lidar_node)
    ld.add_action(camera_node)
    ld.add_action(system_check_ack_node)
    #ld.add_action(plate_detection_node)
    ld.add_action(BufferingNode)
    ld.add_action(PlateDetection)
    ld.add_action(VerificationNode)

    return ld