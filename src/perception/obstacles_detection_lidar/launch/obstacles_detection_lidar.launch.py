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
        'min_height': -2.3,
        'vehicle_height': 2.5,
        'vehicle_length': 3.23,
        'vehicle_width': 1.425,
        
        # 聚类算法相关参数
        'roi_width': 10.0,
        'cluster_tolerance': 0.3,
        'min_cluster_size': 200,
        'max_cluster_size': 10000,
        'leaf_size_x': 0.1,
        'leaf_size_y': 0.1,
        'leaf_size_z': 0.2,
        'plane_point_percent': 0.5,
        


        'is_use_gnss':False,
        
        # 调试参数
        'enable_visualization': True,
        'enable_use_roi': True,
        'enable_calculate_process_time': True,
        'enable_downsample': True,
        'segment_ground_type': 2,
        
        # 相关订阅参数
        'is_use_front_lidar': True,
        'front_lidar_topic': '/rslidar_sdk/drivers/front_lidar',
        'is_use_left_lidar': False,
        'left_lidar_topic': 'drivers/left_lidar',
        'is_use_right_lidar': False,
        'right_lidar_topic': 'drivers/right_lidar',
        'is_use_front_camera': False,
        'front_camera_topic': 'drivers/front_camera',
        
        # 消息ID
        'front_lidar_frame_id': 'lidar_link',
        'base_frame_id': 'base_link',
        'map_frame_id': 'map'
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
