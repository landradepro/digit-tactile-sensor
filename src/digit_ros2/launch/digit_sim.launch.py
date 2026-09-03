from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Defaults assume the TACTO simulation (see simulation/) is running on the
    # same machine - no physical sensor or remote host involved at all.
    stream_host_arg = DeclareLaunchArgument('stream_host', default_value='127.0.0.1')
    stream_port_arg = DeclareLaunchArgument('stream_port', default_value='8091')
    enable_sim_gripper_arg = DeclareLaunchArgument('enable_sim_gripper_bridge', default_value='true')
    sim_gripper_port_arg = DeclareLaunchArgument('sim_gripper_port', default_value='8092')

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

    sim_gripper_node = Node(
        package='digit_ros2',
        executable='sim_gripper_forwarder',
        name='sim_gripper_forwarder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_sim_gripper_bridge')),
        parameters=[{
            'sim_host': LaunchConfiguration('stream_host'),
            'sim_port': ParameterValue(LaunchConfiguration('sim_gripper_port'), value_type=int),
        }],
    )

    return LaunchDescription([
        stream_host_arg,
        stream_port_arg,
        enable_sim_gripper_arg,
        sim_gripper_port_arg,
        base_launch,
        sim_gripper_node,
    ])
