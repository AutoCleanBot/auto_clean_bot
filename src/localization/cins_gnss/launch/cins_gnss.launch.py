import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # 获取包路径和配置文件路径
    package_dir = get_package_share_directory('cins_gnss')
    config_file = os.path.join(package_dir, 'config', 'cins_gnss.param.yaml')
    
    # 配置CINS GNSS节点
    cins_gnss_node = Node(
        package='cins_gnss',
        executable='cins_gnss_node',
        name='cins_gnss_node',               # 节点名称
        output='screen',
        parameters=[config_file],            # 使用YAML配置文件
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        cins_gnss_node
    ])