import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('labzero')
    
    # Define the path to the RViz configuration file
    rviz_config_path = os.path.join(package_dir, 'config', 'circler.rviz2')

    return LaunchDescription([
        # Declare the launch argument for the RViz configuration file path
        DeclareLaunchArgument(
            'circler_rviz_config',
            default_value=rviz_config_path,
            description='Full path to the RVIZ config file'
        ),
        
        # Node definition for the circler node
        Node(
            package='labzero',
            executable='circler',
            name='circler',
            output='screen'
        ),
        
        # Node definition for RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('circler_rviz_config')],
            output='screen'
        ),
    ])