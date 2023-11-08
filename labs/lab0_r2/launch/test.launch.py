from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab0_r2',
            executable='publisher',
            name='publisher',
            output='screen'
        ),
        Node(
            package='lab0_r2',
            executable='subscriber',
            name='subscriber',
            output='screen'
        ),
        # Add any additional nodes or configurations here if necessary.
    ])

