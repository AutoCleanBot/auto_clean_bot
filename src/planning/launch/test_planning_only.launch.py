#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

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
        'test_mode' : True,            # 启用测试模式
        'min_obstacle_distance' : 10.0,  # 最小障碍物距离阈值(米)
        'front_obstacle_width' : 1.0,    # 前方障碍物区域宽度(±米)
        'side_obstacle_width' : 2.0,     # 侧方障碍物区域距离(±米外)
        'occupied_threshold' : 50,        # 占用阈值 (0-100)
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
    
    return LaunchDescription([
        planning_node,
    ])
