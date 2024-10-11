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
        remappings=[('raw_image', '/camera_1/raw_image')],
        parameters=[{
            'camera_device_location': '/dev/video0'
        }]
    )

    camera_hld_node_front_view = Node(
        package='camera_hld',
        executable='camera_hld_node',
        name='camera_hld_camera_front_view',  #Node name (with package name as prefix)
        output='screen', #Log to terminal
        remappings=[('raw_image', '/camera_1/raw_image')],
        parameters=[{
            'tf_frame_id': 'camera_1_front'  # Configurable parameters for this node <param name> : <value>
        }]
    )

    camera_lld_node_side_view = Node(
        package='camera_lld',
        executable='camera_lld_node',
        name='camera_lld_camera_side_view', 
        output='screen',
        remappings=[('raw_image', '/camera_2/raw_image')],
        parameters=[{
            'camera_device_location': '/dev/video2'
        }]
    )

    camera_hld_node_side_view = Node(
        package='camera_hld',
        executable='camera_hld_node',
        name='camera_hld_camera_side_view',
        output='screen', #Log to terminal
        remappings=[('raw_image', '/camera_2/raw_image')],
        parameters=[{
            'tf_frame_id': 'camera_2_side' 
        }]
    )

    return LaunchDescription([
        # camera_lld_node_front_view,
        # camera_hld_node_front_view,
        camera_lld_node_side_view,
        camera_hld_node_side_view
        ])