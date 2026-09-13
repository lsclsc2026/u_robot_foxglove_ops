import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动任务协调器节点"""
    config_file = os.path.join(
        get_package_share_directory('mos_coordinator'),
        'config',
        'coordinator_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='mos_coordinator',
            executable='coordinator_node',
            name='task_coordinator',
            output='screen',
            parameters=[config_file],
        ),
    ])
