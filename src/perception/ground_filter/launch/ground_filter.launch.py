#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ground_filter')
    
    # 参数配置文件路径
    param_file = os.path.join(pkg_dir, 'config', 'ground_filter.param.yaml')
    
    # 启动地面滤波器节点
    ground_filter_node = Node(
        package='ground_filter',
        executable='ground_filter_node',
        name='ground_filter',
        parameters=[
            param_file  # 只使用配置文件，不覆盖
        ],
        output='screen'
    )
    
    return LaunchDescription([
        ground_filter_node
    ]) 