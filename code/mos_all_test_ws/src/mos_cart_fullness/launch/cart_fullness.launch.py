from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("mos_cart_fullness"))
    params = package_share / "config" / "cart_fullness.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "preview",
                default_value="false",
                description="Show OpenCV preview with YOLO boxes and fullness status.",
            ),
            Node(
                package="mos_cart_fullness",
                executable="mos_cart_fullness",
                name="mos_cart_fullness",
                output="screen",
                parameters=[
                    str(params),
                    {
                        "weights": str(package_share / "weights" / "r1g2_a06_0.pt"),
                        "preview": ParameterValue(
                            LaunchConfiguration("preview"),
                            value_type=bool,
                        ),
                    },
                ],
            )
        ]
    )
