import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device_index_arg = DeclareLaunchArgument('device_index', default_value='0')
    image_topic_arg = DeclareLaunchArgument('topic', default_value='digit/image_raw')
    rate_arg = DeclareLaunchArgument('publish_rate_hz', default_value='30.0')
    enable_depth_arg = DeclareLaunchArgument('enable_depth', default_value='true')
    calibration_dir_arg = DeclareLaunchArgument(
        'calibration_dir', default_value=os.path.expanduser('~/digit_ws/calibration_data'))

    camera_node = Node(
        package='digit_ros2',
        executable='camera_publisher',
        name='digit_camera_publisher',
        output='screen',
        parameters=[{
            'device_index': ParameterValue(LaunchConfiguration('device_index'), value_type=int),
            'topic': LaunchConfiguration('topic'),
            'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
        }],
    )

    contact_node = Node(
        package='digit_ros2',
        executable='contact_detector',
        name='contact_detector',
        output='screen',
        parameters=[{'input_topic': LaunchConfiguration('topic')}],
    )

    pressure_node = Node(
        package='digit_ros2',
        executable='pressure_estimator',
        name='pressure_estimator',
        output='screen',
        parameters=[{'input_topic': LaunchConfiguration('topic')}],
    )

    depth_node = Node(
        package='digit_ros2',
        executable='depth_reconstructor',
        name='depth_reconstructor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_depth')),
        parameters=[{
            'input_topic': LaunchConfiguration('topic'),
            'calibration_dir': LaunchConfiguration('calibration_dir'),
        }],
    )

    return LaunchDescription([
        device_index_arg,
        image_topic_arg,
        rate_arg,
        enable_depth_arg,
        calibration_dir_arg,
        camera_node,
        contact_node,
        pressure_node,
        depth_node,
    ])