#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory('ros2_stage')

    simple_world_path = os.path.join(package_dir, 'worlds', 'simple.world')

    ld = LaunchDescription()

    # Declare the launch options
    declare_simple_world_cmd = DeclareLaunchArgument(
        'simple_world',
        default_value=simple_world_path,
        description='Full path to simple.world'
    )

    # Node definition for simple world launch
    node1 = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage_ros2',
        arguments=['-d', LaunchConfiguration('simple_world')],
        output='screen'
    )

    ld.add_action(node1)

    return ld
