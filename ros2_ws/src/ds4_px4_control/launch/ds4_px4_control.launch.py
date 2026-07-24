"""Launch joy_node and the PX4 DS4 position controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ds4_px4_control'),
        'config',
        'ds4.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='Linux joystick device index used by joy_node.',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': ParameterValue(
                    LaunchConfiguration('device_id'), value_type=int),
                'deadzone': 0.0,
                'autorepeat_rate': 50.0,
                'coalesce_interval_ms': 1,
            }],
            output='screen',
        ),
        Node(
            package='ds4_px4_control',
            executable='ds4_position_control',
            name='ds4_position_control',
            parameters=[config],
            output='screen',
        ),
    ])
