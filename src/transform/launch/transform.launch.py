import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('transform')
    
    # 构建配置文件的完整路径
    config_file = os.path.join(pkg_dir, 'config', 'static_transform.yaml')
    
    # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # 配置节点，加载YAML配置文件
    transform_node = Node(
        package='transform',
        executable='transform_node',
        name='transform_node',
        output='screen',
        parameters=[config_file],  # 使用YAML配置文件
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        transform_node
    ]) 