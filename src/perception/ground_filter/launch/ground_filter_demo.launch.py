#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ground_filter')
    
    # 声明话题参数
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/points',
        description='输入点云话题'
    )
    
    ground_points_topic_arg = DeclareLaunchArgument(
        'ground_points_topic',
        default_value='/ground_points',
        description='地面点云输出话题'
    )
    
    no_ground_points_topic_arg = DeclareLaunchArgument(
        'no_ground_points_topic',
        default_value='/no_ground_points',
        description='非地面点云输出话题'
    )
    
    # 包含地面滤波器启动文件
    ground_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ground_filter.launch.py')
        ),
        launch_arguments={
            'input_topic': LaunchConfiguration('input_topic'),
            'ground_points_topic': LaunchConfiguration('ground_points_topic'),
            'no_ground_points_topic': LaunchConfiguration('no_ground_points_topic'),
        }.items()
    )
    
    # 包含RViz可视化启动文件
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'ground_filter_viz.launch.py')
        )
    )
    
    # 启动点云发布测试节点
    test_publisher_node = Node(
        package='ground_filter',
        executable='test_ground_filter.py',
        name='point_cloud_publisher',
        parameters=[
            {'output_topic': LaunchConfiguration('input_topic')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        input_topic_arg,
        ground_points_topic_arg,
        no_ground_points_topic_arg,
        ground_filter_launch,
        rviz_launch,
        test_publisher_node
    ]) 