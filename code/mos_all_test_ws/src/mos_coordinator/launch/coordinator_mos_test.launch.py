#!/usr/bin/env python3
"""Coordinator + Nodes 测试启动文件"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import os


def generate_launch_description():
    """启动 coordinator 和所有 dummy 节点进行集成测试"""

    # 获取配置文件路径
    pkg_dir = get_package_share_directory('mos_coordinator')
    config_file = os.path.join(pkg_dir, 'config', 'coordinator_params.yaml')

    """启动视觉、硬件控制器和任务协调器。

    Nav2 不在此 launch 中启动，应先由外部 bringup 启动。
    """
    object_share = Path(
        get_package_share_directory("mos_3d_object")
    )
    fullness_share = Path(
        get_package_share_directory("mos_cart_fullness")
    )
    grasp_share = Path(
        get_package_share_directory("mos_grasp_selector")
    )

    node_3d_object = Node(
        package="mos_3d_object",
        executable="mos_3d_object_node",
        name="mos_3d_object",
        output="screen",
        parameters=[
            str(object_share / "config" / "yolo_params.yaml"),
            {
                "weights": str(object_share / "weights" / "best3.pt"),
                "static_extrinsic_file": str(
                    object_share / "config" / "static_extrinsic.yaml"
                ),
            },
        ],
    )

    node_cart_fullness = Node(
        package="mos_cart_fullness",
        executable="mos_cart_fullness",
        name="mos_cart_fullness",
        output="screen",
        parameters=[
            str(fullness_share / "config" / "cart_fullness.yaml"),
            {
                "weights": str(fullness_share / "weights" / "best3.pt"),
            },
        ],
    )

    node_grasp_selector = Node(
        package="mos_grasp_selector",
        executable="mos_grasp_selector",
        name="mos_grasp_selector",
        output="screen",
        parameters=[
            str(grasp_share / "config" / "grasp_selector_params.yaml")
        ],
    )

    start_fullness_after_3d = RegisterEventHandler(
        OnProcessStart(
            target_action=node_3d_object,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[node_cart_fullness],
                )
            ],
        )
    )

    start_grasp_after_3d = RegisterEventHandler(
        OnProcessStart(
            target_action=node_3d_object,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[node_grasp_selector],
                )
            ],
        )
    )

    return LaunchDescription([
        # 先启动 3D 检测和硬件控制器。
        node_3d_object,
        Node(
            package="mos_arm_controller",
            executable="mos_hardware_controller_node",
            name="mos_hardware_controller_node",
            output="screen",
        ),

        # 3D 节点启动后再启动满载检测和抓取选择器。
        start_fullness_after_3d,
        start_grasp_after_3d,

        # 等待视觉和硬件节点初始化后启动 Coordinator。
        TimerAction(
            period=30.0,
            actions=[
                Node(
                    package="mos_coordinator",
                    executable="coordinator_node",
                    name="task_coordinator",
                    output="screen",
                )
            ],
        ),
    ])
