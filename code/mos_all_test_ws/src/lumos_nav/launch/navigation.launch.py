import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    nav2_bringup_share_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    pkg_nav = FindPackageShare(package='lumos_nav').find('lumos_nav')

    navigation_launch_file = os.path.join(nav2_bringup_share_dir, 'launch', 'navigation_launch.py')
    rviz_config_dir = os.path.join(pkg_nav, 'rviz', 'view.rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time if true, system time if false')

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value='nav2_params.yaml',

        description='Nav2 params file (nav2_params_fastlio.yaml for FAST-LIO)')
    declare_rviz_cmd = DeclareLaunchArgument(
        name='rviz', default_value='true',
        description='Launch RViz')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map_yaml',
        default_value='/opt/slam/map_datas/maps/pgo_map.yaml',
        description='Absolute path to the 2D occupancy map yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map_yaml')
    rviz = LaunchConfiguration('rviz')

    nav2_params_path = PathJoinSubstitution([pkg_nav, 'params', params_file])

    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'params_file': nav2_params_path,
            'use_sim_time': use_sim_time,
            'map': map_yaml,
        }.items()
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(rviz_node)
    ld.add_action(start_navigation_cmd)

    return ld
