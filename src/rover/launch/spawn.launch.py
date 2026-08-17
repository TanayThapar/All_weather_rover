from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


import xacro


def generate_launch_description():

    pkg = get_package_share_directory('rover')
    xacro_file = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'rover_world.sdf')

    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'all_weather_explorer',
                '-topic', 'robot_description',
                '-z', '0.5'
            ],
            output='screen'
        ),
    ])
