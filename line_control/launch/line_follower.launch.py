import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('line_control'),
        'config',
        'params.yaml',
    )

    return LaunchDescription([
        Node(
            package='line_vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='line_control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[params_file],
        ),
    ])