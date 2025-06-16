import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # 将所有参数放在一个字典中
    planning_params = {
        'local_topic_name':'/localization/rtk_info',
        'service_name' : '/routing_service',
        'traj_topic_name' : '/planning/trajectory',
        'perc_topic_name' : '/perception/obstacles',
        'path_type' : 2,
        'process_frq' : 10.0,    # 处理频率, Hz
        'preview_dist' : 12.0,   # 预览距离, m
        'preview_time' : 1.0,     # 预瞄时间, s
        'start_dist' : 5.0,      # 起始距离, m
        'traj_pub_interval' : 0.1, # 路径发布间隔, 秒
        'planning_spd' : 2.0,     # 规划速度, m/s
        'reverse_moving' : False  # 是否反向行驶
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',  # 保持与代码中一致
        output='screen',
        parameters=[planning_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        planning_node
    ])
