import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    fast_lio_localization_pkg_path = get_package_share_directory('fast_lio_localization')
    rviz = LaunchConfiguration('rviz')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_localization_pkg_path, 'launch', 'lidar_and_localization.launch.py')
        ),
        launch_arguments={
            'rviz': rviz,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        SetEnvironmentVariable('SLAM_LIDAR1_NAME', 'MID360'),
        localization_launch,
    ])
