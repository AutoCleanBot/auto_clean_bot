# import launch
# import launch.actions
# import launch.substitutions
# import launch_ros.actions


# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch.actions.DeclareLaunchArgument(
#             'node_prefix',
#             default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
#             description='Prefix for node names'),
#         launch_ros.actions.Node(
#             package='canbus', executable='can_driver', output='screen',
#             name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'can_driver']),
#     ])

#!/usr/bin/python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canbus',
            executable='can_driver',
            name='node_can',
            output='screen'
        )
    ])
