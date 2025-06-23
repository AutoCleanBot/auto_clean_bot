#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Include the main test launch file
    test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('obstacle_marker_test'),
                'launch',
                'obstacle_marker_test.launch.py'
            ])
        ]),
        launch_arguments={
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }.items()
    )

    # RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('obstacle_marker_test'),
        'config',
        'obstacle_marker_test.rviz'
    ])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        frame_id_arg,
        publish_rate_arg,
        use_rviz_arg,
        test_launch,
        rviz_node
    ])
