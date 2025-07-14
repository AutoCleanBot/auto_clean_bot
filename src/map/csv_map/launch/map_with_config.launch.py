import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def expand_tilde_in_config(config_file_path):
    """读取配置文件并展开其中的波浪号路径"""
    try:
        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)

        # 展开 map_files_dir 中的波浪号
        if 'map_node' in config and 'ros__parameters' in config['map_node']:
            params = config['map_node']['ros__parameters']
            if 'map_files_dir' in params:
                params['map_files_dir'] = os.path.expanduser(params['map_files_dir'])

        return config
    except Exception as e:
        print(f"Error reading config file {config_file_path}: {e}")
        return None

def generate_launch_description():
    # 获取地图包的目录
    map_pkg_dir = get_package_share_directory('csv_map')

    # 配置文件路径
    config_file_path = LaunchConfiguration('config_file')
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(map_pkg_dir, 'config', 'map_config.yaml'),
        description='Path to the map configuration file'
    )

    # 读取并处理配置文件
    default_config_path = os.path.join(map_pkg_dir, 'config', 'map_config.yaml')
    config = expand_tilde_in_config(default_config_path)

    # 配置地图节点
    if config:
        map_node = Node(
            package='csv_map',
            executable='map_node',
            name='map_node',
            output='screen',
            parameters=[config['map_node']['ros__parameters']]
        )
    else:
        # 如果配置文件读取失败，使用默认参数
        map_node = Node(
            package='csv_map',
            executable='map_node',
            name='map_node',
            output='screen',
            parameters=[{
                'map_files_dir': os.path.expanduser('~/auto_clean_bot/map_files'),
                'left_boundary_file': 'local_record_2_left_boundary.csv',
                'right_boundary_file': 'local_record_2_right_boundary.csv',
                'left_boundary_name': 'left_boundary',
                'right_boundary_name': 'right_boundary',
                'boundary_length': 30.0,
                'publish_frequency': 10.0
            }]
        )

    return LaunchDescription([
        declare_config_file,
        map_node
    ])