from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("mos_route_manager"))
    params = package_share / "config" / "route_manager.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "goal_file",
                default_value="/opt/slam/config/target_goals.yaml",
                description="Named map-frame navigation goals YAML.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="Start patrol automatically only after all required services are ready.",
            ),
            Node(
                package="mos_route_manager",
                executable="mos_route_manager",
                name="mos_route_manager",
                output="screen",
                parameters=[
                    str(params),
                    {
                        "goal_file": LaunchConfiguration("goal_file"),
                        "auto_start": ParameterValue(
                            LaunchConfiguration("auto_start"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
