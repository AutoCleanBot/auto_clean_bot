import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取地图文件目录的默认路径
    default_map_files_dir = os.path.join(os.environ['HOME'], 'auto_clean_bot', 'map_files')
    
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
            'right_boundary_name': 'right_boundary'
        }]
    )
    
    return LaunchDescription([
        declare_map_files_dir,
        declare_left_boundary_file,
        declare_right_boundary_file,
        declare_boundary_length,
        declare_publish_frequency,
        map_node
    ]) 