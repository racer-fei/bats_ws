
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'map': '/path/to/map.yaml'
            }]
        )
    ])
