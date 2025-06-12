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
    remote_controller_params = {
        'can_device_name': 'can1',
        'can_baudrate': 250,
        'remote_controller_topic': '/remote_controller_topic',
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    remote_controller_node = Node(
        package='remote_controller',
        executable='remote_controller_node',
        name='remote_controller_node',  # 保持与代码中一致
        output='screen',
        parameters=[remote_controller_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        remote_controller_node
    ])
