from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('rover')
    urdf = os.path.join(pkg, 'urdf', 'rover.urdf')
    world_file = os.path.join(pkg, 'worlds', 'rover_world.sdf')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf).read(),
                'use_sim_time': True
            }],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'rover',
                '-file', urdf
            ],
            output='screen'
        ),
    ])
