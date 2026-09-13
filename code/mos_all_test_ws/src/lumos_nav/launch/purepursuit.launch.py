import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('lumos_nav'),
        'params',
        'purepursuit_follower_params.yaml'
    )

    purepursuit_node = Node(
        package='lumos_nav',
        executable='purepursuit_follower',
        name='purepursuit_follower',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        purepursuit_node
    ])
