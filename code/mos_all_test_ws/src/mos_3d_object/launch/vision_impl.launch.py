from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("mos_3d_object"))
    params = package_share / "config" / "yolo_params.yaml"

    return LaunchDescription(
        [
            Node(
                package="mos_3d_object",
                executable="mos_3d_object_node",
                name="mos_3d_object",
                output="screen",
                parameters=[
                    str(params),
                    {
                        "weights": str(package_share / "weights" / "r1g2_a06_0.pt"),
                        "static_extrinsic_file": str(
                            package_share / "config" / "static_extrinsic.yaml"
                        )
                    },
                ],
            )
        ]
    )
