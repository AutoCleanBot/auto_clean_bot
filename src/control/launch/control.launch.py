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
    control_params = {
        'publish_rate': 10.0,
        'preview_time': 10.0,
        'tolerance_distance': 0.1,
        'max_steering_angle':30.0,
        'wheelbase':1.2,
        'max_linear_velocity':10.0,
        'max_angular_velocity':10.0,
        'acceleration_limit':1.0,
        'deceleration_limit':-1.0,
        'adc_traj_topic_name':'planning/adc_traj',
        'localization_info_topic_name':'localization/rtk_info',
        'control_cmd_topic_name':'control/control_cmd'
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',  # 保持与代码中一致
        output='screen',
        parameters=[control_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        control_node
    ])
