#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('costmap_generator')
    
    # 配置文件路径
    config_file = os.path.join(pkg_dir, 'config', 'costmap_generator.param.yaml')
    

    
    # 启动代价地图生成器节点
    costmap_generator_node = Node(
        package='costmap_generator',
        executable='costmap_node',
        name='costmap_generator',
        parameters=[
            config_file
        ],
        output='screen'
    )
    

    
    return LaunchDescription([
        costmap_generator_node,
    ]) 