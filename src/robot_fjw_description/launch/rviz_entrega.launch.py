"""RViz para entrega: mapa SLAM + letras planificadas y recorridas."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    paquete = get_package_share_directory('robot_fjw_description')
    ruta = os.path.join(paquete, 'config', 'fjw_entrega.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_entrega',
            output='screen',
            arguments=['-d', ruta],
            parameters=[{'use_sim_time': True}],
        )
    ])
