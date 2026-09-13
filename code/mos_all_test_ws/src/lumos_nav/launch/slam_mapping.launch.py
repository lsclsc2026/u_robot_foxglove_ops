import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    fast_lio_pkg_path = get_package_share_directory('fast_lio')
    livox_pkg_path = get_package_share_directory('livox_ros_driver2')
    pkg_nav = get_package_share_directory('lumos_nav')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg_path, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'rviz': 'true',
        }.items()
    )

    livox_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg_path, 'launch', 'msg_MID360_launch.py')
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('SLAM_LIDAR1_NAME', 'MID360'),
        livox_lidar_launch,
        mapping_launch,
    ])
