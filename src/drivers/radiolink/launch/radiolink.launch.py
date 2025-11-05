import os
from ament_index_python import get_package_share_directory
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

    # 获取包路径
    radiolink_pkg_dir = get_package_share_directory('radiolink')
    # 构建yaml配置文件路径
    config_file_path = os.path.join(radiolink_pkg_dir, 'config', 'radio_node.yaml')
    #config_file_path = os.path.join(radiolink_pkg_dir, 'config', 'rtk_with_logging.yaml')

    # 配置节点，从yaml文件加载参数
    radio_node = Node(
        package='radiolink',
        executable='radio_node',
        name='radio_node',  # 保持与代码中一致
        output='screen',
        parameters=[config_file_path],  # 从yaml文件加载参数
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        radio_node
    ])
