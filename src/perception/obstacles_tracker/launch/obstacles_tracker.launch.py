from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = FindPackageShare('obstacles_tracker')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'obstacles_tracker.yaml'])
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    # 创建障碍物跟踪器节点
    obstacles_tracker_node = Node(
        package='obstacles_tracker',
        executable='obstacles_tracker_node',
        name='obstacles_tracker_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        obstacles_tracker_node
    ]) 