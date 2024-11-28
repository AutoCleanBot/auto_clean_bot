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
    rtk_params = {
        'device_name' : '/dev/ttyUART_232_A',
        'baud_rate' : 460800,
        'timeout_ms' : 10,
        'base_longtitude':117.38,  #基点经度
        'base_latitude': 31.96,  #基点纬度
        'base_altitude': 0.0,  #基点高度
        
        'local_topic_name' : 'localization/rtk_info',
        'local_frame_id' : 'rtk_link',
        'local_publish_rate' : 10.0,  # 频率为10Hz
        'imu_topic_name' : 'imu/pose',
        'imu_frame_id' : 'imu_link',
        'imu_publish_rate' : 50.0,  # 频率为50Hz
        'gnss_topic_name' : 'gnss/pose',
        'gnss_frame_id' : 'world_link',
        'gnss_publish_rate' : 10.0,  # 频率为10Hz
        'enable_debug_log': False,
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    rtk_node = Node(
        package='rtk',
        executable='rtk_node',
        name='rtk_node',  # 保持与代码中一致
        output='screen',
        parameters=[rtk_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        rtk_node
    ])
