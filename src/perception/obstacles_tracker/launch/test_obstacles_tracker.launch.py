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
    
    # 创建测试发布器节点
    test_publisher_node = Node(
        package='obstacles_tracker',
        executable='test_publisher_node',
        name='test_publisher_node',
        output='screen'
    )
    
    # 创建障碍物跟踪器节点
    obstacles_tracker_node = Node(
        package='obstacles_tracker',
        executable='obstacles_tracker_node',
        name='obstacles_tracker_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    # 创建RViz节点用于可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'obstacles_tracker_view.rviz'])]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        test_publisher_node,
        obstacles_tracker_node,
        rviz_node
    ]) 