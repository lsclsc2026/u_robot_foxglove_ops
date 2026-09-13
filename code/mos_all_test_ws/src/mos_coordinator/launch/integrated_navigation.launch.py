#!/usr/bin/env python3
"""完整集成启动：IMU滤波 + EKF + FAST-LIO + Nav2 + 协调器"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """先启动完整导航栈，再启动协调器"""
    pkg_nav = get_package_share_directory('lumos_nav')
    pkg_coord = get_package_share_directory('mos_coordinator')

    # 导航 bringup（IMU滤波 + FAST-LIO + EKF + Nav2）
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2_fastlio_bringup.launch.py')
        ),
        launch_arguments={
            'rviz': 'true',
        }.items()
    )

    # 协调器（Nav2 就绪后启动，使用真实 NavigateToPose action）
    config_file = os.path.join(pkg_coord, 'config', 'coordinator_params.yaml')
    coordinator_node = Node(
        package='mos_coordinator',
        executable='coordinator_node',
        name='task_coordinator',
        output='screen',
        parameters=[config_file],
    )

    # 延迟启动协调器，等 Nav2 lifecycle 完全激活
    delayed_coordinator = TimerAction(
        period=15.0,
        actions=[coordinator_node],
    )

    return LaunchDescription([
        nav_bringup,
        delayed_coordinator,
    ])
