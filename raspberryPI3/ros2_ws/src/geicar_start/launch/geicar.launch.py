from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joystick",
        executable="joystick_ros2.py",
        emulate_tty=True
    )

    joystick_to_cmd_node = Node( 
        package="joystick",
        executable="joystick_to_cmd",
        emulate_tty=True
    )

    can_rx_node = Node(
        package="can",
        executable="can_rx_node",
        emulate_tty=True
    )

    can_tx_node = Node(
        package="can",
        executable="can_tx_node",
        emulate_tty=True
    )

    car_control_node = Node(
        package="car_control",
        executable="car_control_node",
        emulate_tty=True
    )

    config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
        emulate_tty=True
    )
    
    imu_frame_id_modifier_node = Node(
        package="imu_filter_madgwick",
        executable="imu_frame_id_modifier_node",
        emulate_tty=True
    )

    system_check_node = Node(
        package="system_check",
        executable="system_check_node",
        emulate_tty=True
    )

    rosbridge_server_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        emulate_tty=True
    )

    obstacles_detection_node = Node(
        package="detection",
        executable="obstacles_detection_node",
        emulate_tty=True
    )

    localization_config_dir = os.path.join(get_package_share_directory('geicar_start'), 'config')

        # Get the path to the secondary launch file
    launch_dir = os.path.join(get_package_share_directory('geicar_start'),'launch')
    
    navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navsat.launch.py'))
    ),
    
    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py'))
    ),

    
    path_publisher_node = Node(
        package='path_smoother',
        executable='path_publisher',
        output="screen",
    )

    pure_pursuit_planner_node = Node(
        package='pure_pursuit_planner',
        executable='pure_pursuit_planner',
        output="screen",
    )   
    
    mqtt_client_node = Node(
        package="mqtt_client",
        executable="mqtt_client",
        emulate_tty=True
    )

    ld.add_action(mqtt_client_node)
    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(car_control_node)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(imu_frame_id_modifier_node)
    ld.add_action(system_check_node)
    ld.add_action(obstacles_detection_node)
    ld.add_action(navsat_launch)
    ld.add_action(sensor_fusion_launch)
    ld.add_action(path_publisher_node)
    ld.add_action(pure_pursuit_planner_node)
    
    return ld
