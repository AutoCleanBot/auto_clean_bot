#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    can1_device = LaunchConfiguration('can1_device')
    declare_can1_device = DeclareLaunchArgument(
        'can1_device',
        default_value='can1',
        description='CAN device name'
    )
    
    can1_baud = LaunchConfiguration('can1_baud')
    declare_can1_baud = DeclareLaunchArgument(
        'can1_baud',
        default_value='250',
        description='CAN baudrate'
    )
    
    # Create remote controller node
    remote_controller_node = Node(
        package='remote_controller',
        executable='remote_controller_node',
        name='remote_controller_node',
        output='screen',
        parameters=[{
            'can1_device': can1_device,
            'can1_baud': can1_baud,
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        declare_can1_device,
        declare_can1_baud,
        remote_controller_node
    ])
