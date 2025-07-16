from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # RTK模拟器节点 - 使用默认参数
    rtk_simulator_node = Node(
        package='rtk_simulator',
        executable='rtk_simulator_node',
        name='rtk_simulator_node',
        output='screen',
        parameters=[{
            'csv_file_path': '~/auto_clean_bot/path/local_record_2.csv',
            'publish_frequency': 10.0,
            'noise_min_percentage': 1.0,
            'noise_max_percentage': 5.0,
            'loop_trajectory': True,
            'gnss_topic_name': '/gnss/pose',
            'gnss_frame_id': 'map',
        }]
    )
    
    return LaunchDescription([
        rtk_simulator_node
    ])
