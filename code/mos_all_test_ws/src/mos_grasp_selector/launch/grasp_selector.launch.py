from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("mos_grasp_selector"))
    params = package_share / "config" / "grasp_selector_params.yaml"

    return LaunchDescription(
        [
            Node(
                package="mos_grasp_selector",
                executable="mos_grasp_selector",
                name="mos_grasp_selector",
                output="screen",
                parameters=[str(params)],
            )
        ]
    )
