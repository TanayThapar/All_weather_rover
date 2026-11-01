import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover_share = get_package_share_directory('rover')
    default_config = os.path.join(rover_share, 'config', 'fusion_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to fusion node configuration yaml'
    )

    fusion_node = Node(
        package='rover',
        executable='fusion_node',
        name='rover_fusion_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_arg,
        fusion_node
    ])
