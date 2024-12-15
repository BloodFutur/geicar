from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


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

    robot_localization_node = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[os.path.join(localization_config_dir, 'ekf_config.yaml')],
        remappings=[('odometry/filtered', 'odometry/local')]
    ) 

    navsat_transform_node = Node(
        package='robot_localization', 
        executable='navsat_transform_node', 
        name='navsat_transform',
        output='screen',
        parameters=[os.path.join(localization_config_dir, 'ekf_test.yaml')],
        remappings=[('imu/data', '/imu/modified_frame_id'),
                    ('gps/fix', 'gps/fix'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps')]           
    )   

    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(car_control_node)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(imu_frame_id_modifier_node)
    ld.add_action(system_check_node)
    ld.add_action(rosbridge_server_node)
    ld.add_action(obstacles_detection_node)
    ld.add_action(robot_localization_node)
    ld.add_action(navsat_transform_node)

    return ld
