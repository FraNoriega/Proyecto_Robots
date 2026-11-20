from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('robot_fjw_description')
    return LaunchDescription([Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'fjw.rviz')],
        parameters=[{'use_sim_time': True}])])
