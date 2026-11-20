"""Start Gazebo Sim, spawn the original CAD and bridge ROS/Gazebo topics."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('robot_fjw_description')
    gz_pkg = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'fjw_lab.sdf')
    robot_xacro = os.path.join(pkg, 'urdf', 'ROBOT.xacro')
    bridge_yaml = os.path.join(pkg, 'config', 'ros_gz_bridge_gazebo.yaml')
    model = xacro.process_file(robot_xacro).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 3 ' + world}.items(),
    )
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     output='screen', parameters=[{'robot_description': model, 'use_sim_time': True}])
    create = TimerAction(period=5.0, actions=[
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', '/robot_description', '-name', 'ROBOT',
                        '-x', '-4.45', '-y', '-1.25', '-z', '0.005', '-Y', '1.57079632679'])
    ])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}])
    return LaunchDescription([gazebo, publisher, bridge, create])
