import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    device_index_arg = DeclareLaunchArgument('device_index', default_value='0')

    digit_launch_path = os.path.join(
        get_package_share_directory('digit_ros2'), 'launch', 'digit.launch.py')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(digit_launch_path),
        launch_arguments={
            'device_index': LaunchConfiguration('device_index'),
            'enable_depth': 'true',
        }.items(),
    )

    return LaunchDescription([device_index_arg, base_launch])
