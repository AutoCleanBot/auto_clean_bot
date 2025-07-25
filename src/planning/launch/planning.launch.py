from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    # 声明配置文件参数
    config_file = LaunchConfiguration('config_file')
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='planning_params.yaml',
        description='Planning configuration file name'
    )

    # 构建配置文件的完整路径
    config_file_path = PathJoinSubstitution([
        FindPackageShare('planning'),
        'config',
        config_file
    ])

    # 配置节点，使用YAML配置文件
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',  # 保持与代码中一致
        output='screen',
        parameters=[config_file_path],  # 使用YAML配置文件
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        declare_config_file,
        planning_node
    ])
