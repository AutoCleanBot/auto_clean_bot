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
    canbus_params = {
        'can_device': 'can1',
        'can_baud': 5000,
        'control_cmd_topic': '/control_cmd',
        'chassis_info_topic': '/chassis_info_topic',
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    canbus_node = Node(
        package='canbus',
        executable='canbus_node',
        name='canbus_node',  # 保持与代码中一致
        output='screen',
        parameters=[canbus_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        canbus_node
    ])
