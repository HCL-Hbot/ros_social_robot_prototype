from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    camera_lld_node_front_view = Node(
        package='camera_lld',
        executable='camera_lld_node',
        name='camera_lld_camera_front_view',  #Node name (with package name as prefix)
        output='screen',
        parameters=[{
            'usb_port': '/dev/ttyUSB0'
        }]
    )

    camera_hld_node_front_view = Node(
        package='camera_hld',
        executable='camera_hld_node',
        name='camera_hld_camera_front_view',  #Node name (with package name as prefix)
        output='screen', #Log to terminal
        parameters=[{
            'tf_frame_id': 'camera_frame'  # Configurable parameters for this node <param name> : <value>
        }]
    )

    return LaunchDescription([
        camera_lld_node_front_view,
        camera_hld_node_front_view
        ])