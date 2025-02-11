import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #This is camera hld + camera lld
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('camera_hld'), 'launch', 'camera_launch.py')
        )
    )

    interaction_controller_node = Node(
        package='interaction_controller',
        executable='interaction_controller_node',
        name='interaction_controller',
        output='screen'
    )

    eye_display_hld_node = Node(
        package='eye_display_hld',
        executable='eye_display_hld_node',
        name='eye_display_hld',
        output='screen'
    )

    radar_presence_hld_node = Node(
        package='radar_presence_hld',
        executable='radar_presence_hld_node',
        name='radar_presence_hld',
        output='screen'
    )

    return LaunchDescription([
        camera_launch,
        interaction_controller_node,
        eye_display_hld_node,
        radar_presence_hld_node
    ])