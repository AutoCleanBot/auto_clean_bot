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
    radio_params = {
        'device_name': '/dev/ttysWK1',
        'baud_rate': 100000,
        'timeout_ms': 10,

        'radio_topic_name': 'radio/radio_info',
        'radio_frame_id': 'radio_link',
        'radio_publish_rate': 50.0,  # 频率为50Hz

        'enable_debug_log': False,
        'log_interval': 30,            # 日志输出间隔（每25次解析输出一次，适合50Hz频率）

        # info_str 文件保存配置 (新增调试功能)
        'enable_info_str_save': False,        # 启用info_str保存功能 (默认关闭)
        'info_str_save_dir': '/home/nvidia/radio_logs',  # 保存目录
    }

    # 配置节点，并将参数字典直接传递给参数字段
    radio_node = Node(
        package='radiolink',
        executable='radio_node',
        name='radio_node',  # 保持与代码中一致
        output='screen',
        parameters=[radio_params],  # 直接使用参数字典
        # 小的动态测试
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        radio_node
    ])
