import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from datetime import datetime


def generate_launch_description():
    # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    workspace_dir = os.path.expanduser(
        '~/auto_clean_bot/running_logs/')  # 扩展~为用户主目录
    log_dir = os.path.join(workspace_dir, 'control_log')
    # 确保日志目录存在
    if (not os.path.exists(log_dir)):
        os.makedirs(log_dir, exist_ok=True)
    now = datetime.now()
    timestamp_str = now.strftime('%Y%m%d_%H%M%S')
    default_log_path = os.path.join(
        log_dir, f'control_debug_{timestamp_str}.csv')

    declare_log_file_path = DeclareLaunchArgument(
        'log_file_path',
        default_value=default_log_path,
        description='Path to the control debug log file'
    )

    # 获取control包的配置文件路径
    control_pkg_dir = get_package_share_directory('control')
    config_file = os.path.join(control_pkg_dir, 'config', 'control_params.yaml')

    # 配置节点，从yaml文件加载参数
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',  # 保持与代码中一致
        output='screen',
        parameters=[config_file, {'log_file_path': default_log_path}],  # 从yaml文件加载参数
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        control_node
    ])
