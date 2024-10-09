from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_lld',
            executable='camera_lld_node',
            name='camera_lld_node',
            output='screen'
        ),
        Node(
            package='camera_hld',
            executable='camera_hld_node',
            name='camera_hld_node',
            output='screen'
        )
    ])
