#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('costmap_generator')
    
    # 包含代价地图生成器启动文件
    costmap_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'costmap_generator.launch.py')
        )
    )
    
    # 包含RViz可视化启动文件
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'costmap_viz.launch.py')
        )
    )
    
    return LaunchDescription([
        costmap_generator_launch,
        rviz_launch
    ]) 