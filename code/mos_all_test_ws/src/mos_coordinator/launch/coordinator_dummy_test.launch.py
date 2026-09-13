#!/usr/bin/env python3
"""Coordinator + Dummy Nodes 测试启动文件"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 coordinator 和所有 dummy 节点进行集成测试"""

    # 获取配置文件路径
    pkg_dir = get_package_share_directory('mos_coordinator')
    config_file = os.path.join(pkg_dir, 'config', 'coordinator_params.yaml')

    return LaunchDescription([
        # ===== Coordinator 节点 =====
        Node(
            package='mos_coordinator',
            executable='coordinator_node',
            name='coordinator',
            output='screen',
            parameters=[config_file],
            emulate_tty=True,
        ),

        # ===== Dummy Vision 节点 =====
        
        # Dummy 3D 物体检测节点 (模拟 mos_3d_object)
        Node(
            package='mos_coordinator',
            executable='dummy_3d_object_node',
            name='dummy_mos_3d_object',
            output='screen',
            parameters=[{
                'enabled': True,
                'poll_seconds': 3.0,  # 每3秒发布一次检测
                'camera': 'head',
                'target_frame': 'base_link',
                'detections_3d_topic': '/vision/detections_3d',
            }],
            emulate_tty=True,
        ),

        # Dummy 购物车满度检测节点 (模拟 mos_cart_fullness)
        Node(
            package='mos_coordinator',
            executable='dummy_cart_fullness_node',
            name='dummy_mos_cart_fullness',
            output='screen',
            parameters=[{
                'enabled': True,
                'camera_poll_seconds': 0.5,
                'waypoint_topic': '/cart_fullness/set_waypoint',
                'route_signal_topic': '/cart_fullness/route_signal',
                'status_topic': '/cart_fullness/status',
            }],
            emulate_tty=True,
        ),

        # Dummy 抓取点选择节点 (模拟 mos_grasp_selector)
        Node(
            package='mos_coordinator',
            executable='dummy_grasp_selector_node',
            name='dummy_mos_grasp_selector',
            output='screen',
            parameters=[{
                'enabled': True,
                'detections_topic': '/vision/detections_3d',
                'grasp_tasks_topic': '/vision/grasp_tasks',
                'required_points': 2,
                'min_stable_frames': 3,
            }],
            emulate_tty=True,
        ),

        # ===== Dummy Chassis 节点（底盘对准）=====
        Node(
            package='mos_coordinator',
            executable='dummy_chassis_node',
            name='dummy_chassis',
            output='screen',
            parameters=[{
                # MoveChassis 服务相关参数可以在这里配置
            }],
            emulate_tty=True,
        ),

        # ===== Dummy Arm 节点 =====
        Node(
            package='mos_coordinator',
            executable='dummy_arm_node',
            name='dummy_arm',
            output='screen',
            parameters=[{
                # RunTask 服务相关参数可以在这里配置
            }],
            emulate_tty=True,
        ),

        # ===== Dummy Navigator 节点 =====
        Node(
            package='mos_coordinator',
            executable='dummy_navigator_node',
            name='dummy_navigator',
            output='screen',
            parameters=[{
                'success_rate': 0.9,        # 导航成功率 90%
                'min_delay': 0.5,           # 最小延迟 0.5 秒
                'max_delay': 2.0,           # 最大延迟 2.0 秒
                'goal_topic': '/goal_pose',
                'status_topic': '/navigator/status',
                'localization_topic': '/localization',
            }],
            emulate_tty=True,
        ),
    ])
