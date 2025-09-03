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
    
    # 配置remote_controller_simulator节点
    simulator_node = Node(
        package='remote_controller_simulator',
        executable='remote_controller_simulator_node',
        name='remote_controller_simulator',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        simulator_node
    ])