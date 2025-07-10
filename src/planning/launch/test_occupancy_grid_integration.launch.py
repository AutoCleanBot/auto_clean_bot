#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 规划节点参数
    planning_params = {
        'local_topic_name':'/localization/rtk_info',
        'service_name' : '/routing_service',
        'traj_topic_name' : '/planning/trajectory',
        'perc_topic_name' : '/perception/obstacles',
        'left_boundary_topic_name' : '/map/left_boundary',
        'right_boundary_topic_name' : '/map/right_boundary',
        'occupancy_grid_topic_name' : '/occupancy_grid',  # 占用栅格地图话题
    'use_occupancy_grid' : True,   # 使用占用栅格地图进行障碍物检测
        'path_type' : 2,
        'process_frq' : 10.0,    # 处理频率, Hz
        'preview_dist' : 12.0,   # 预览距离, m
        'preview_time' : 1.0,     # 预瞄时间, s
        'start_dist' : 5.0,      # 起始距离, m
        'traj_pub_interval' : 0.1, # 路径发布间隔, 秒
        'planning_spd' : 3.0,     # 规划速度, m/s
        'path_end_dist' : 2.0,    # 路径结束距离, m
        'reverse_moving' : False  # 是否反向行驶
    }
    
    # 规划节点
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        parameters=[planning_params],
        output='screen'
    )
    
    # 测试数据发布节点
    test_publisher = ExecuteProcess(
        cmd=['python3', '/home/limer/auto_clean_bot/src/planning/test_occupancy_grid_integration.py'],
        output='screen'
    )
    
    return LaunchDescription([
        test_publisher,
        planning_node,
    ])
