from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    localization_config_dir = os.path.join(get_package_share_directory('geicar_start'), 'config')

    local_localization_node = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[os.path.join(localization_config_dir, 'ekf_config.yaml')],
        remappings=[('odometry/filtered', 'odometry/localronan')]
    ) 
    
    global_localization_node = Node(
        package='robot_localization', 
        executable='ekf_node', 
        name='ekf_filter_node_map',
	    output='screen',
        parameters=[os.path.join(localization_config_dir, 'ekf_config.yaml')],
        remappings=[('odometry/filtered', 'odometry/globalronan')]
    )
    return LaunchDescription([
        local_localization_node,
        global_localization_node
    ])
