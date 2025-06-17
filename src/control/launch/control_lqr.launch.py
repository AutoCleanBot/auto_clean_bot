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
    default_log_path = os.path.join(log_dir, f'control_lqr_debug_{timestamp_str}.csv')

    declare_log_file_path = DeclareLaunchArgument(
        'log_file_path',
        default_value=default_log_path,
        description='Path to the control debug log file'
    )
    
    # 将所有参数放在一个字典中
    control_params = {
        'publish_rate': 50.0,
        'preview_time': 1.5,            # 增加预瞄时间
        'tolerance_distance': 1.5,      # 增加预瞄距离容差
        'max_steering_angle': 40.0,     # 减小最大前轮转角
        'wheelbase': 1.99,              # 设置为估计轴距
        'max_linear_velocity': 8.0,     # 减小最大线速度
        'max_angular_velocity': 8.0,    # 减小最大角速度
        'acceleration_limit': 15.0,     # 减小加速度限制
        'deceleration_limit': -15.0,    # 减小减速度限制
        'pursuit_control_rate': 0.6,    # 增加纯追踪控制比例
        'stanley_control_rate': 0.4,    # 减少Stanley控制比例
        'sta_lat_rate': 0.4,            # 减小横向误差系数
        'feedforward_rate': 1.0,        # 增加前馈控制比例
        'speed_pid_kp': 0.4,            # 减小速度PID比例系数
        'speed_pid_ki': 0.08,           # 减小速度PID积分系数
        'speed_pid_kd': 0.05,           # 增加速度PID微分系数
        'zero_point_draft': -3.0,       # 零点漂移
        'turning_radius_ratio': 1.0,
        'max_speed_change_rate': 0.8,   # 减小最大速度变化率
        'smooth_window_size': 25,       # 增加平滑窗口大小
        'adc_traj_topic_name': '/planning/trajectory',
        'localization_info_topic_name': '/localization/rtk_info',
        'control_cmd_topic_name': '/control/control_cmd',
        
        # LQR控制器参数
        'use_lqr_controller': True,     # 启用LQR控制器
        'cf': 250000.0,                 # 前轮侧偏刚度
        'cr': 250000.0,                 # 后轮侧偏刚度
        'mass': 4050.0,                 # 车辆质量
        'iz': 7000.0,                   # 车辆转动惯量
        'lqr_max_iterations': 150,      # LQR最大迭代次数
        'lqr_eps': 0.01,                # LQR收敛容差
        
        # 状态权重矩阵Q
        'Q(0, 0)': 40.0,                # 增大横向误差权重
        'Q(1, 1)': 5.0,                 # 增大横向误差变化率权重
        'Q(2, 2)': 30.0,                # 增大航向误差权重
        'Q(3, 3)': 2.0,                 # 增大航向误差变化率权重
        
        # 控制权重矩阵R
        'R(0, 0)': 30.0                 # 增大控制输入权重，使控制更平滑
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
        declare_log_file_path,
        control_node
    ]) 