#!/usr/bin/env python3

from launch.actions.declare_launch_argument import DeclareLaunchArgument


from launch.actions.declare_launch_argument import DeclareLaunchArgument


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lidar_imu_init')
    default_config_path = os.path.join(package_share, 'config', 'robosense.yaml')
    default_rviz_config = os.path.join(package_share, 'rviz_cfg', 'spinning.rviz')

    rviz_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    config_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        'config',
        default_value=default_config_path,
        description='Path to lidar_imu_init parameter YAML file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file'
    )

    lidar_imu_node = Node(
        package='lidar_imu_init',
        executable='li_init',
        name='laserMapping',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'point_filter_num': 3,
                'max_iteration': 5,
                'cube_side_length': 2000.0,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        config_arg,
        rviz_config_arg,
        lidar_imu_node,
        rviz_node,
    ])
