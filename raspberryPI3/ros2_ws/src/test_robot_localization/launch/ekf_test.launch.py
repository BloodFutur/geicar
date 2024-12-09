from launch import LaunchDescription
import launch_ros.actions
import os
import launch.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('test_robot_localization'), 'config')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),
	
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[os.path.join(config_dir, 'ekf_test.yaml')],
            remappings=[('odometry/filtered', 'odometry/local')]           
           ),
    # launch_ros.actions.Node(
    #         package='robot_localization', 
    #         executable='ekf_node', 
    #         name='ekf_filter_node_map',
	#         output='screen',
    #         parameters=[os.path.join(config_dir, 'ekf_test.yaml')],
    #         remappings=[('odometry/filtered', 'odometry/global')]
    #        ),           
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[os.path.join(config_dir, 'ekf_test.yaml')],
            remappings=[('imu/data', 'imu'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps')]           

           )           
    ])