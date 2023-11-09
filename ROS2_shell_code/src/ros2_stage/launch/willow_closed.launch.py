#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory('ros2_stage')

    world_path = os.path.join(package_dir, 'worlds', 'willow_closed.world')

    ld = LaunchDescription()

    # Declare the launch options
    declare_simple_world_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=world_path,
        description='Full path to the world file'
    )

    gazebo_backend = Node(
        package='gazebo_ros', # This is the ROS package for Gazebo integration
        executable='gzserver', # This is the Gazebo server executable
        arguments=['-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world_file')],
        output='screen'
    )

    gazebo_executable = Node(
        package='gazebo_ros',
        executable='gzclient', # This is the Gazebo client executable
        output='screen'
    )

    ld.add_action(gazebo_backend)
    ld.add_action(gazebo_executable)

    return ld
