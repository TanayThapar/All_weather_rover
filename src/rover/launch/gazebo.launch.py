import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    rover_package = get_package_share_directory('rover')

    xacro_file = os.path.join(
        rover_package,
        'urdf',
        'rover.urdf.xacro'
    )

    gazebo_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    world_file = os.path.join(
        rover_package,
        'worlds',
        'rover_world.sdf'
    )

    # ============================================================
    # Process Xacro -> URDF XML
    # ============================================================

    robot_description = xacro.process_file(xacro_file).toxml()

    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # Start Gazebo with rover_world.sdf
        # --------------------------------------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={
                'gz_args': '-r ' + world_file
            }.items()
        ),

        # --------------------------------------------------------
        # Robot State Publisher
        # --------------------------------------------------------

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': True
                }
            ]
        ),

        # --------------------------------------------------------
        # Spawn rover into Gazebo
        # --------------------------------------------------------

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic',
                'robot_description',

                '-name',
                'all_weather_explorer',

                '-x',
                '0',

                '-y',
                '0',

                '-z',
                '0.5',

                '-Y',
                '3.14159'
            ],
            output='screen'
        ),

        # --------------------------------------------------------
        # ROS Gz Bridge
        # --------------------------------------------------------

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/all_weather_explorer/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/world/default/model/all_weather_explorer/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[
                ('/model/all_weather_explorer/odometry', '/odom'),
                ('/world/default/model/all_weather_explorer/joint_state', '/joint_states'),
            ],
            output='screen'
        ),
    ])
