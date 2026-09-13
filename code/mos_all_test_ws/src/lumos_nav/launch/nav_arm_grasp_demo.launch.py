from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("lumos_nav"))
    default_params = package_share / "config" / "nav_arm_grasp_demo_params.yaml"

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Parameter file for nav + arm grasp demo",
            ),
            Node(
                package="lumos_nav",
                executable="nav_arm_grasp_demo.py",
                name="nav_arm_grasp_demo",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
