from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mos_chassis_controller",
                executable="mos_chassis_controller_node",
                name="mos_chassis_controller",
                output="screen",
            )
        ]
    )
