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
    routing_params = {
        # 路径名与其对应的文件路径, 最多支持10个路径
        '1':'/home/limer/auto_clean_bot/path/local_record.csv',
        '2':'/home/limer/auto_clean_bot/path/global_record.csv',
        '3':'/home/limer/auto_clean_bot/path/obstacle_record.csv',
        '4':'/home/limer/auto_clean_bot/path/path_record.csv',
        '5':'/home/limer/auto_clean_bot/path/path_record_2.csv',
        '6':'/home/limer/auto_clean_bot/path/path_record_3.csv',
        '7':'/home/limer/auto_clean_bot/path/path_record_4.csv',
        '8':'/home/limer/auto_clean_bot/path/path_record_5.csv',
        '9':'/home/limer/auto_clean_bot/path/path_record_6.csv',
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    routing_node = Node(
        package='routing',
        executable='routing_node',
        name='routing_node',  # 保持与代码中一致
        output='screen',
        parameters=[routing_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        routing_node
    ])
