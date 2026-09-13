import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('lumos_nav')
    fast_lio_localization_pkg_path = get_package_share_directory('fast_lio_localization')

    config_path = os.path.join(pkg_nav, 'params', 'purepursuit_follower_params.yaml')
    arrival_monitor_config_path = os.path.join(
        pkg_nav, 'params', 'goal_arrival_monitor_params.yaml'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_localization_pkg_path, 'launch', 'lidar_and_localization.launch.py')
        )
    )

    tf_bridge_node = Node(
        package='lumos_nav',
        executable='localization_tf_bridge.py',
        name='localization_tf_bridge',
        output='screen',
        parameters=[{
            'livox_to_base.x': -0.061,
            'livox_to_base.y': 0.0,
            'livox_to_base.z': 0.0,
            'livox_to_base.qx': 1.0,
            'livox_to_base.qy': 0.0,
            'livox_to_base.qz': 0.0,
            'livox_to_base.qw': 0.0,
        }],
    )

    cloud_stamp_bridge_node = Node(
        package='lumos_nav',
        executable='cloud_stamp_bridge.py',
        name='cloud_stamp_bridge',
        output='screen',
    )

    scan_frame_bridge_node = Node(
        package='lumos_nav',
        executable='scan_frame_bridge.py',
        name='scan_frame_bridge',
        output='screen',
    )

    purepursuit_node = Node(
        package='lumos_nav',
        executable='purepursuit_follower',
        name='purepursuit_follower',
        output='screen',
        parameters=[config_path],
    )

    arrival_monitor_node = Node(
        package='lumos_nav',
        executable='goal_arrival_monitor.py',
        name='goal_arrival_monitor',
        output='screen',
        parameters=[arrival_monitor_config_path],
    )

    wait_for_map_to_odom_node = Node(
        package='lumos_nav',
        executable='wait_for_map_to_odom.py',
        name='wait_for_map_to_odom',
        output='screen',
    )

    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            localization_launch,
            tf_bridge_node,
            cloud_stamp_bridge_node,
            scan_frame_bridge_node,
            wait_for_map_to_odom_node,
        ]
    )

    start_follower_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_to_odom_node,
            on_exit=[purepursuit_node, arrival_monitor_node],
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('SLAM_LIDAR1_NAME', 'MID360'),
        delayed_nodes,
        start_follower_when_ready,
    ])
