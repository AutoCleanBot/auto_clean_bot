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
    rtk_params = {
        'device_name' : '/dev/ttyTHS4',
        'baud_rate' : 460800,
        'timeout_ms' : 50,
        'topic_name' : 'rtk_fix',
        'frame_id' : 'rtk_link',
        'publish_rate' : 50.0,  # 频率为10Hz
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    rtk_node = Node(
        package='rtk',
        executable='rtk_node',
        name='rtk_node',  # 保持与代码中一致
        output='screen',
        parameters=[rtk_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        rtk_node
    ])
