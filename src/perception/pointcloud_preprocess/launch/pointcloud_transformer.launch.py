#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('pointcloud_preprocess')
    
    # 参数配置文件路径
    param_file = os.path.join(pkg_dir, 'config', 'pointcloud_transformer.param.yaml')
    
    # 启动点云坐标转换节点
    pointcloud_transformer_node = Node(
        package='pointcloud_preprocess',
        executable='pointcloud_transformer_node',
        name='pointcloud_transformer',
        parameters=[
            param_file
        ],
        output='screen'
    )
    
    return LaunchDescription([
        pointcloud_transformer_node
    ]) 