#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ground_filter')
    
    # RViz配置文件路径
    rviz_config_file = os.path.join(pkg_dir, 'config', 'ground_filter_viz.rviz')
    
    # 启动RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_node
    ]) 