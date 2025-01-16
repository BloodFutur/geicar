from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'magnetic_declination_radians': 0.0},
                {'yaw_offset': 0.0},
                {'zero_altitude': False},
                {'base_link_frame': 'base_link'},
                {'world_frame': 'odom'},
            ],
            remappings=[
                ('/imu', '/imu/modified_frame_id'),
                ('/gps/fix', '/gps/fix'),
                ('/odometry/filtered', '/odometry/globalronan'),
                ('/odometry/gps', '/odometry/gps'),
            ],
            arguments=[
                '--ros-args', '--log-level', 'debug'
            ]
        )
    ])
