from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_lld',
            name='camera_lld',
            output='screen'
        ),
        Node(
            package='camera',
            executable='camera_hld',
            name='camera_hld',
            output='screen'
        )
    ])
