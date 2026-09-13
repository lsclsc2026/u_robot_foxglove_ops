import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_name = os.environ.get('SLAM_LIDAR1_NAME', 'MID360')
    os.environ['SLAM_LIDAR1_NAME'] = lidar_name

    pkg_nav = get_package_share_directory('lumos_nav')
    fast_lio_localization_pkg = get_package_share_directory('fast_lio_localization')
    arrival_monitor_params = os.path.join(
        pkg_nav, 'params', 'goal_arrival_monitor_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map_yaml')
    pcd_map = LaunchConfiguration('pcd_map')
    localization_config_file = LaunchConfiguration('localization_config_file')
    auto_load_pose = LaunchConfiguration('auto_load_pose')
    pose_validity_check = LaunchConfiguration('pose_validity_check')
    max_pose_age_seconds = LaunchConfiguration('max_pose_age_seconds')
    rviz = LaunchConfiguration('rviz')
    enable_direct_base_controller = LaunchConfiguration('enable_direct_base_controller')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_localization_pkg, 'launch', 'lidar_and_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'map2d': map_yaml,
            'pcd_map': pcd_map,
            'config_path': os.path.join(pkg_nav, 'config', 'fast_lio'),
            'config_file': localization_config_file,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map_yaml': map_yaml,
            'rviz': 'false',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav, 'config', 'nav2_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )

    tf_bridge_node = Node(
        package='lumos_nav',
        executable='localization_tf_bridge.py',
        name='localization_tf_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    scan_frame_bridge_node = Node(
        package='lumos_nav',
        executable='scan_frame_bridge.py',
        name='scan_frame_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    cmd_vel_to_base_node = Node(
        package='lumos_nav',
        executable='cmd_vel_to_base.py',
        name='cmd_vel_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_direct_base_controller),
    )

    goal_arrival_monitor_node = Node(
        package='lumos_nav',
        executable='goal_arrival_monitor.py',
        name='goal_arrival_monitor',
        output='screen',
        parameters=[arrival_monitor_params, {'use_sim_time': use_sim_time}],
    )

    wait_for_map_to_odom_node = Node(
        package='lumos_nav',
        executable='wait_for_map_to_odom.py',
        name='wait_for_map_to_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_navigation_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_to_odom_node,
            on_exit=[navigation_launch],
        )
    )

    delayed_bringup = TimerAction(
        period=2.0,
        actions=[
            localization_launch,
            tf_bridge_node,
            cloud_stamp_bridge_node,
            scan_frame_bridge_node,
            cmd_vel_to_base_node,
            goal_arrival_monitor_node,
            wait_for_map_to_odom_node,
            start_navigation_when_ready,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='nav2_params_fastlio.yaml',
            description='Nav2 params file in lumos_nav/params'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='/opt/slam/map_datas/maps/pgo_map.yaml',
            description='Absolute path to the 2D occupancy map yaml'
        ),
        DeclareLaunchArgument(
            'pcd_map',
            default_value='/opt/slam/map_datas/maps/map.pcd',
            description='Absolute path to the PCD map used by FAST-LIO localization'
        ),
        DeclareLaunchArgument(
            'localization_config_file',
            default_value='localization_mid360.yaml',
            description='FAST-LIO localization config file in lumos_nav/config/fast_lio'
        ),
        DeclareLaunchArgument(
            'auto_load_pose',
            default_value='true',
            description='Reuse /opt/slam/map_datas/maps/last_pose.json as initial pose'
        ),
        DeclareLaunchArgument(
            'pose_validity_check',
            default_value='true',
            description='Reject cached poses that fail basic sanity checks'
        ),
        DeclareLaunchArgument(
            'max_pose_age_seconds',
            default_value='3600',
            description='Maximum age of cached pose before falling back to manual input'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch a single Nav2 RViz instance'
        ),
        DeclareLaunchArgument(
            'enable_direct_base_controller',
            default_value='false',
            description=(
                'Run lumos_nav cmd_vel_to_base.py. Keep false when '
                'mos_arm_controller owns physical chassis commands.'
            ),
        ),
        SetEnvironmentVariable('SLAM_LIDAR1_NAME', lidar_name),
        SetEnvironmentVariable('SLAM_AUTO_LOAD_POSE', auto_load_pose),
        SetEnvironmentVariable('SLAM_POSE_VALIDITY_CHECK', pose_validity_check),
        SetEnvironmentVariable('SLAM_MAX_POSE_AGE', max_pose_age_seconds),
        ExecuteProcess(
            cmd=["bash", "-c", "pkill -f 'gicp_localization|transform_fusion|fastlio_mapping|livox_ros_driver2|pointcloud_to_laserscan|cloud_stamp_bridge|wait_for_map_to_odom|cmd_vel_to_base|map_server|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|localization_tf_bridge' 2>/dev/null; sleep 1; echo done"],
            name='cleanup_nav2_processes',
            shell=False,
        ),
        rviz_node,
        delayed_bringup,
    ])
