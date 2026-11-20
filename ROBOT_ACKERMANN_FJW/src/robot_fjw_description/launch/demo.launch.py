"""Una orden: inicia Gazebo, RViz y el controlador de las letras F-J-W."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_fjw_description')

    def lanzamiento(nombre):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', nombre)))

    rviz = lanzamiento('rviz_fjw.launch.py')
    controlador = Node(
        package='robot_fjw_control',
        executable='ejecutar_fjw',
        name='controlador_fjw',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'control_fjw.yaml')],
    )
    return LaunchDescription([
        lanzamiento('gazebo.launch.py'),
        TimerAction(period=7.0, actions=[rviz, controlador]),
    ])
