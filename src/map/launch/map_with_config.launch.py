import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取地图包的目录
    map_pkg_dir = get_package_share_directory('map')
    
    # 配置文件路径
    config_file = LaunchConfiguration('config_file')
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(map_pkg_dir, 'config', 'map_config.yaml'),
        description='Path to the map configuration file'
    )
    
    # 配置地图节点
    map_node = Node(
        package='map',
        executable='map_node',
        name='map_node',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        declare_config_file,
        map_node
    ]) 