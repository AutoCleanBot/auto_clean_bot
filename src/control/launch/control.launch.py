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
    
    # 声明日志文件路径参数
    log_file_path = LaunchConfiguration('log_file_path')
    control_share_dir = get_package_share_directory('control')
    log_dir = os.path.join(control_share_dir, 'log')
    now = datetime.now()
    timestamp_str = now.strftime('%Y%m%d_%H%M%S')
    default_log_path = os.path.join(log_dir, f'control_debug_{timestamp_str}.csv')

    declare_log_file_path = DeclareLaunchArgument(
        'log_file_path',
        default_value=default_log_path,
        description='Path to the control debug log file'
    )
    
    # 将所有参数放在一个字典中
    control_params = {
        'publish_rate': 50.0,
        'preview_time': 1.0,            # 预瞄时间
        'tolerance_distance': 4.0,      # 预瞄距离
        'max_steering_angle':50.0,      # 最大前轮转角
        'wheelbase':1.4,
        'max_linear_velocity':10.0,
        'max_angular_velocity':10.0,
        'acceleration_limit':5.0,
        'deceleration_limit':-5.0,
        'pursuit_control_rate':1.0,
        'stanley_control_rate':0.8,
        'sta_lat_rate':0.4, # stanley控制中的横向偏差系数
        'speed_pid_kp':0.5,
        'speed_pid_ki':0.1,
        'speed_pid_kd':0.0,
        'zero_point_draft':0.0,     # 零点漂移
        'turning_radius_ratio':1.0,
        'adc_traj_topic_name':'/planning/trajectory',
        'localization_info_topic_name':'/localization/rtk_info',
        'control_cmd_topic_name':'/control/control_cmd',
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',  # 保持与代码中一致
        output='screen',
        parameters=[control_params,
                    {'log_file_path': default_log_path}],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        control_node
    ])
