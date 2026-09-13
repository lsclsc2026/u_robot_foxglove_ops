from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mos_arm_controller",
                executable="mos_hardware_controller_node",
                name="mos_hardware_controller",
                output="screen",
            )
        ]
    )
