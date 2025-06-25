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
    checking_params = {
        'source_frame': 'lidar_link',  # 源坐标系
        'target_frame': 'base_link',   # 目标坐标系
        'check_period': 1.0,           # 检查周期（秒）
    }
    
    # 配置节点
    checking_node = Node(
        package='transform_checking',
        executable='transform_checking_node',
        name='transform_checking_node',
        output='screen',
        parameters=[checking_params],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        checking_node
    ]) 