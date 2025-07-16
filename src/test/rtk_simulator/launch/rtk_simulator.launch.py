import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('rtk_simulator')

    # 声明配置文件参数
    config_file = LaunchConfiguration('config_file')
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'rtk_simulator.yaml'),
        description='Path to the RTK simulator configuration file'
    )

    # RTK模拟器节点 - 仅使用配置文件
    rtk_simulator_node = Node(
        package='rtk_simulator',
        executable='rtk_simulator_node',
        name='rtk_simulator_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        declare_config_file,
        rtk_simulator_node
    ])
