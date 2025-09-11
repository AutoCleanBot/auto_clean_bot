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
    
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('data_recorder'),
        'config',
        'data_recorder.yaml'
    )
    
    # 数据记录节点
    data_recorder_node = Node(
        package='data_recorder',
        executable='data_recorder_node',
        name='data_recorder_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        data_recorder_node
    ])