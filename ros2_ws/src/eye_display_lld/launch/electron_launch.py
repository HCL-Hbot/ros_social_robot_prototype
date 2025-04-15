from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def launch_setup(context, *args, **kwargs):
    # Get argument values from launch context
    mode = LaunchConfiguration('mode').perform(context)
    left_eye = LaunchConfiguration('left_eye').perform(context)
    right_eye = LaunchConfiguration('right_eye').perform(context)

    # Path to the Electron app (always points to source folder)
    electron_app_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '../../../../../src/eye_display_lld/src')
    )

    # Prepare CLI flags only if values are given
    extra_args = []
    if left_eye:
        extra_args.append(f'--left-eye={left_eye}')
    if right_eye:
        extra_args.append(f'--right-eye={right_eye}')

    # Build final command
    cmd = ['npm', 'run', mode, '--'] + extra_args

    return [
        LogInfo(msg=f'Launching Electron app in mode="{mode}" with args: {" ".join(extra_args)}'),
        ExecuteProcess(
            cmd=cmd,
            cwd=electron_app_path,
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='start', description='Run mode: start or dev'),
        DeclareLaunchArgument('left_eye', default_value='', description='Left eye screen index (0 or 1)'),
        DeclareLaunchArgument('right_eye', default_value='', description='Right eye screen index (0 or 1)'),

        OpaqueFunction(function=launch_setup)
    ])
