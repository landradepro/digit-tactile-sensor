import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # DIGIT is plugged directly into this machine's USB - no network streaming needed.
    device_index_arg = DeclareLaunchArgument('device_index', default_value='0')
    # Off by default: depth_reconstructor crashes without calibration data, which a
    # fresh clone won't have yet. Enable once you've done the calibration in the README.
    enable_depth_arg = DeclareLaunchArgument('enable_depth', default_value='false')

    digit_launch_path = os.path.join(
        get_package_share_directory('digit_ros2'), 'launch', 'digit.launch.py')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(digit_launch_path),
        launch_arguments={
            'device_index': LaunchConfiguration('device_index'),
            'enable_depth': LaunchConfiguration('enable_depth'),
        }.items(),
    )

    return LaunchDescription([device_index_arg, enable_depth_arg, base_launch])
