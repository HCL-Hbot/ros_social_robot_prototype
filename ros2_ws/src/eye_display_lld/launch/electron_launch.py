from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the launch argument for the mode (default: 'start')
    mode = LaunchConfiguration('mode')

    # Resolve the path to the Electron app (relative to this launch file in the install folder)
    electron_app_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '../../../../../src/eye_display_lld/src')
    )

    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'mode',
            default_value='start',
            description='Mode to run the Electron app: "start" for production, "dev" for development which includes hot-reloading'
        ),

        # Log the mode being used
        LogInfo(msg=["Launching Electron app in mode: ", mode]),

        # Start the Electron app with npm
        ExecuteProcess(
            cmd=['npm', 'run', mode],
            cwd=electron_app_path,
            output='screen'
        )
    ])
