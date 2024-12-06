from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ndt_mapping')
    config_file = os.path.join(pkg_share, 'config', 'ndt_params.yaml')

    # Create node action
    ndt_mapping_node = Node(
        package='ndt_mapping',
        executable='ndt_mapping_node',
        name='ndt_mapping_node',
        parameters=[
            config_file
        ],
        output='screen',
    )

    return LaunchDescription([
        ndt_mapping_node
    ])