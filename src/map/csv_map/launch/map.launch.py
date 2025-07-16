import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取地图文件目录的默认路径 (展开波浪号)
    default_map_files_dir = os.path.expanduser("~/auto_clean_bot/map_files")
    
    # 声明启动参数
    map_files_dir = LaunchConfiguration('map_files_dir')
    declare_map_files_dir = DeclareLaunchArgument(
        'map_files_dir',
        default_value=default_map_files_dir,
        description='Directory containing map files'
    )
    
    left_boundary_file = LaunchConfiguration('left_boundary_file')
    declare_left_boundary_file = DeclareLaunchArgument(
        'left_boundary_file',
        default_value='local_record_4_left_boundary.csv',
        description='Left boundary file name'
    )
    
    right_boundary_file = LaunchConfiguration('right_boundary_file')
    declare_right_boundary_file = DeclareLaunchArgument(
        'right_boundary_file',
        default_value='local_record_4_right_boundary.csv',
        description='Right boundary file name'
    )
    
    boundary_length = LaunchConfiguration('boundary_length')
    declare_boundary_length = DeclareLaunchArgument(
        'boundary_length',
        default_value='50.0',
        description='Length of boundary segment to publish'
    )
    
    publish_frequency = LaunchConfiguration('publish_frequency')
    declare_publish_frequency = DeclareLaunchArgument(
        'publish_frequency',
        default_value='10.0',
        description='Frequency to publish boundary information'
    )

    # 方向稳定性参数
    direction_stability_weight = LaunchConfiguration('direction_stability_weight')
    declare_direction_stability_weight = DeclareLaunchArgument(
        'direction_stability_weight',
        default_value='2.0',
        description='Direction stability weight for boundary selection'
    )

    max_index_jump = LaunchConfiguration('max_index_jump')
    declare_max_index_jump = DeclareLaunchArgument(
        'max_index_jump',
        default_value='30.0',
        description='Maximum allowed index jump for boundary points'
    )

    yaw_weight = LaunchConfiguration('yaw_weight')
    declare_yaw_weight = DeclareLaunchArgument(
        'yaw_weight',
        default_value='3.0',
        description='Yaw difference weight for boundary selection'
    )
    
    # 配置地图节点
    map_node = Node(
        package='csv_map',
        executable='map_node',
        name='map_node',
        output='screen',
        parameters=[{
            'map_files_dir': map_files_dir,
            'left_boundary_file': left_boundary_file,
            'right_boundary_file': right_boundary_file,
            'boundary_length': boundary_length,
            'publish_frequency': publish_frequency,
            'left_boundary_name': 'left_boundary',
            'right_boundary_name': 'right_boundary',
            'direction_stability_weight': direction_stability_weight,
            'max_index_jump': max_index_jump,
            'yaw_weight': yaw_weight
        }]
    )
    
    return LaunchDescription([
        declare_map_files_dir,
        declare_left_boundary_file,
        declare_right_boundary_file,
        declare_boundary_length,
        declare_publish_frequency,
        declare_direction_stability_weight,
        declare_max_index_jump,
        declare_yaw_weight,
        map_node
    ])