"""Optional SLAM Toolbox launch; only use after Gazebo publishes /scan & TF."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('robot_fjw_description')
    slam_pkg = get_package_share_directory('slam_toolbox')
    params = os.path.join(pkg, 'config', 'slam.yaml')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': params, 'use_sim_time': 'true'}.items())])
