from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_arguments = [
        DeclareLaunchArgument(
            'node_name',
            default_value='default_camera_node',
            description='Name of the camera node'
        ),
        # DeclareLaunchArgument(
        #     'usb_port',
        #     default_value='/dev/video0',
        #     description='USB port for the camera'
        # ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_frame',
            description='Frame ID for the camera data'
        )
    ]

    # Node configuratie
    # camera_lld_node = Node(
    #     package='camera_lld',
    #     executable='camera_lld_node',
    #     name=LaunchConfiguration('node_name'),
    #     output='screen',
    #     parameters=[{
    #         'node_name': LaunchConfiguration('node_name'),
    #         'usb_port': LaunchConfiguration('usb_port')
    #     }]
    # )

    camera_hld_node = Node(
        package='camera_hld',
        executable='camera_hld_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[{
            'node_name': LaunchConfiguration('node_name'),
            'frame_id': LaunchConfiguration('frame_id')
        }]
    )

    return LaunchDescription(launch_arguments + [camera_hld_node])

    #return LaunchDescription(launch_arguments + [camera_lld_node, camera_hld_node])
