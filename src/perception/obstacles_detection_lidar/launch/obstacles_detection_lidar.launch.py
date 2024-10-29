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
    obstacles_detection_params = {
        # 传感器以及车辆参数
        'max_height': 3.0,
        'min_height': 0.0,
        'vehicle_height': 0.4,
        'vehicle_length': 0.4,
        'vehicle_width': 0.4,
        
        # 聚类算法相关参数
        'roi_width': 1.0,
        'cluster_tolerance': 0.4,
        'min_cluster_size': 20,
        'max_cluster_size': 30000,
        'leaf_size': 0.1,
        'plane_point_percent': 0.5,
        
        # 坐标系转换参数
        'lidar_base_x': 0.0,
        'lidar_base_y': 0.0,
        'lidar_base_z': 0.0,
        'lidar_base_yaw': 180.0,
        'lidar_base_pitch': 0.0,
        'lidar_base_roll': 0.0,
        'radar_base_x': 0.0,
        'radar_base_y': 0.0,
        'radar_base_z': 0.0,
        'radar_base_yaw': 0.0,
        'radar_base_pitch': 0.0,
        'radar_base_roll': 0.0,
        
        # 调试参数
        'enable_visualization': False,
        'enable_use_roi': False,
        'enable_calculate_process_time': False,
        'enable_downsample': True,
        'segment_ground_type': 1,
        
        # 相关订阅参数
        'is_use_front_lidar': True,
        'front_lidar_topic': 'drivers/front_lidar',
        'is_use_left_lidar': False,
        'left_lidar_topic': 'drivers/left_lidar',
        'is_use_right_lidar': False,
        'right_lidar_topic': 'drivers/right_lidar',
        'is_use_front_camera': False,
        'front_camera_topic': 'drivers/front_camera',
        
        # 消息ID
        'frame_id': 'base_link'
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    obstacles_detection_lidar_node = Node(
        package='obstacles_detection_lidar',
        executable='obstacles_detection_lidar_node',
        name='obstacles_detection_lidar_node',  # 保持与代码中一致
        output='screen',
        parameters=[obstacles_detection_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        obstacles_detection_lidar_node
    ])
