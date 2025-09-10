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
    local_record_params = {
        'save_path': '~/auto_clean_bot/path/local_record',
        'save_rate': 10.0,
        'topic_name': '/localization/rtk_info',
        'file_number': 21,
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    local_record_node = Node(
        package='local_record',
        executable='local_record_node',
        name='local_record_node',  # 保持与代码中一致
        output='screen',
        parameters=[local_record_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        local_record_node
    ])
