import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    env_default = os.environ.get('DIGIT_STREAM_HOST')
    if env_default is not None:
        stream_host_arg = DeclareLaunchArgument(
            'stream_host', default_value=env_default,
            description='IP address of the Windows PC running windows_stream_server.py '
                         '(defaulted from $DIGIT_STREAM_HOST)')
    else:
        stream_host_arg = DeclareLaunchArgument(
            'stream_host',
            description='IP address of the Windows PC running windows_stream_server.py '
                         '(tip: export DIGIT_STREAM_HOST=<ip> to avoid retyping this)')

    stream_port_arg = DeclareLaunchArgument('stream_port', default_value='8090')

    digit_launch_path = os.path.join(
        get_package_share_directory('digit_ros2'), 'launch', 'digit.launch.py')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(digit_launch_path),
        launch_arguments={
            'stream_host': LaunchConfiguration('stream_host'),
            'stream_port': LaunchConfiguration('stream_port'),
            'enable_depth': 'false',
        }.items(),
    )

    return LaunchDescription([stream_host_arg, stream_port_arg, base_launch])
