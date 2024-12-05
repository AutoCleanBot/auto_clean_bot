from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ndt_mapping')
    config_file = os.path.join(pkg_share, 'config', 'ndt_params.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    points_topic_arg = DeclareLaunchArgument(
        'points_topic',
        default_value='points_raw',
        description='Input pointcloud topic'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu_raw',
        description='Input IMU topic'
    )

    # Create node action
    ndt_mapping_node = Node(
        package='ndt_mapping',
        executable='ndt_mapping_node',
        name='ndt_mapping_node',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'points_topic': LaunchConfiguration('points_topic'),
                'imu_topic': LaunchConfiguration('imu_topic')
            }
        ],
        output='screen',
        remappings=[
            ('points_raw', LaunchConfiguration('points_topic')),
            ('/imu_raw', LaunchConfiguration('imu_topic'))
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        points_topic_arg,
        imu_topic_arg,
        ndt_mapping_node
    ])