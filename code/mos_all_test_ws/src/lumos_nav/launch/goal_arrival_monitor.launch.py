import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('lumos_nav')
    params = os.path.join(pkg_nav, 'params', 'goal_arrival_monitor_params.yaml')

    return LaunchDescription([
        Node(
            package='lumos_nav',
            executable='goal_arrival_monitor.py',
            name='goal_arrival_monitor',
            output='screen',
            parameters=[params],
        )
    ])
