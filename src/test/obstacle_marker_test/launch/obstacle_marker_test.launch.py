#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Frame ID for the markers'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Publishing rate in Hz'
    )
    
    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('obstacle_marker_test'),
        'config',
        'obstacle_marker_test.yaml'
    ])
    
    # Create the test node
    test_node = Node(
        package='obstacle_marker_test',
        executable='obstacle_marker_test_node',
        name='obstacle_marker_test_node',
        parameters=[
            config_file,
            {
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        frame_id_arg,
        publish_rate_arg,
        test_node
    ])
