from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    audio_manager_node = Node(
        package='audio_hld',
        executable='audio_manager_node',
        name='audio_manager',
        output='screen'
    )

    audio_file_player_node = Node(
        package='audio_lld',
        executable='audio_file_player_node',
        name='audio_file_player',  #Node name (with package name as prefix)
        output='screen'
        
        #Example of setting parameters for this node. No parameters = autodetect mode.
        #parameters=[{
        #    'device_device': 'hw:1,0',
        #    'sample_rate': 44100,
        #}]
    )

    return LaunchDescription([
        audio_manager_node,
        audio_file_player_node
        ])