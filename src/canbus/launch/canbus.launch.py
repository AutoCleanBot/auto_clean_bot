# import launch
# import launch.actions
# import launch.substitutions
# import launch_ros.actions


# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch.actions.DeclareLaunchArgument(
#             'node_prefix',
#             default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
#             description='Prefix for node names'),
#         launch_ros.actions.Node(
#             package='canbus', executable='can_driver', output='screen',
#             name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'can_driver']),
#     ])

#!/usr/bin/python3

import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import  DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def get_params_path(package_name, params_file):
    package_dir = get_package_share_directory(package_name)
    if not package_dir:
        raise RuntimeError(f"Failed to get the share directory of the {package_name} package. "
                           "Ensure the package is installed correctly.")
    return os.path.join(package_dir, "config", params_file)

def generate_launch_description():
    params = get_params_path('canbus', 'canbus_conf_dev.yaml')
    if not os.path.isfile(params):
        raise FileNotFoundError(f"The configuration file {params} does not exist.")
    canbus_ff = Node(
        package="canbus",
        executable="can_driver",
        name="node_can",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[params])
    return LaunchDescription([
        DeclareLaunchArgument("namespace",
                              default_value="",
                              description="Namespace for the node"),
        canbus_ff
    ])
