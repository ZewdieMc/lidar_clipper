import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_bag_path',
            default_value='/home/zed/thesis_ws/src/bagFiles/saxion_around',
            description='Path to the input bag file'
        ),
        DeclareLaunchArgument(
            'output_bag_path',
            default_value='/home/zed/thesis_ws/src/bagFiles/saxion_fig8path',
            description='Path to the output bag file'
        ),
        DeclareLaunchArgument(
            'min_z',
            default_value='-30.0',
            description='Minimum z value for clipping'
        ),
        DeclareLaunchArgument(
            'max_z',
            default_value='10.0',
            description='Maximum z value for clipping'
        ),
        Node(
            package='lidar_clipper',
            executable='lidar_clipper_node',
            name='lidar_clipper_node',
            output='screen',
            parameters=[{
                'input_bag_path': LaunchConfiguration('input_bag_path'),
                'output_bag_path': LaunchConfiguration('output_bag_path'),
                'min_z': LaunchConfiguration('min_z'),
                'max_z': LaunchConfiguration('max_z'),
            }]
        )
    ])