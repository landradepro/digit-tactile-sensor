import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device_index_arg = DeclareLaunchArgument('device_index', default_value='0')
    stream_host_arg = DeclareLaunchArgument('stream_host', default_value='')
    stream_port_arg = DeclareLaunchArgument('stream_port', default_value='8090')
    image_topic_arg = DeclareLaunchArgument('topic', default_value='digit/image_raw')
    rate_arg = DeclareLaunchArgument('publish_rate_hz', default_value='30.0')
    enable_depth_arg = DeclareLaunchArgument('enable_depth', default_value='true')
    enable_grasp_decision_arg = DeclareLaunchArgument('enable_grasp_decision', default_value='true')
    target_pressure_arg = DeclareLaunchArgument('target_pressure', default_value='350.0')
    slip_drop_ratio_arg = DeclareLaunchArgument('slip_drop_ratio', default_value='0.3')
    calibration_dir_arg = DeclareLaunchArgument(
        'calibration_dir', default_value=os.path.expanduser('~/digit_ws/calibration_data'))

    camera_node = Node(
        package='digit_ros2',
        executable='camera_publisher',
        name='digit_camera_publisher',
        output='screen',
        parameters=[{
            'device_index': ParameterValue(LaunchConfiguration('device_index'), value_type=int),
            'stream_host': LaunchConfiguration('stream_host'),
            'stream_port': ParameterValue(LaunchConfiguration('stream_port'), value_type=int),
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

    grasp_decision_node = Node(
        package='digit_ros2',
        executable='grasp_decision',
        name='grasp_decision',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_grasp_decision')),
        parameters=[{
            'target_pressure': ParameterValue(LaunchConfiguration('target_pressure'), value_type=float),
            'slip_drop_ratio': ParameterValue(LaunchConfiguration('slip_drop_ratio'), value_type=float),
        }],
    )

    return LaunchDescription([
        device_index_arg,
        stream_host_arg,
        stream_port_arg,
        image_topic_arg,
        rate_arg,
        enable_depth_arg,
        enable_grasp_decision_arg,
        target_pressure_arg,
        slip_drop_ratio_arg,
        calibration_dir_arg,
        camera_node,
        contact_node,
        pressure_node,
        depth_node,
        grasp_decision_node,
    ])
