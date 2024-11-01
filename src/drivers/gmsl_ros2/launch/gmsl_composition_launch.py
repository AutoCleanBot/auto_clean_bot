"""
Launch a ComposableNode with parameters and remappings.

Limitations:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Set parameters
namespace = 'camera'
frame_id = 'gmsl_camera_frame'
camera_name = 'gmsl_camera'
camera_config = 'file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', 'calibration_param_example.yaml')

def generate_launch_description():

    camera_dev = LaunchConfiguration('camera_dev', default='/dev/video0')
    camera_type = LaunchConfiguration('camera_type', default='argus')
    index = LaunchConfiguration('index', default='0')
    # width = LaunchConfiguration('width', default='2048')
    # height = LaunchConfiguration('height', default='1280')
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')
    open_rviz = LaunchConfiguration('open_rviz', default='false')

    # v4l2 camera node
    container_v4l2 = ComposableNodeContainer(
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "v4l2"'])),
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gmsl_ros2',
                plugin='gscam2::GSCamNode',
                name='gmsl_publisher',
                namespace=namespace,
                parameters=[
                    {
                        'gst_config': (['v4l2src device=', camera_dev, ' ! video/x-raw,format=(string)UYVY,framerate=30/1,width=', width,',height=', height,' ! videoconvert ! video/x-raw, format=(string)BGR ! videoconvert']),
                        'preroll': False,
                        'use_gst_timestamps': False,
                        'frame_id': frame_id,
                        'camera_name': camera_name,
                        'camera_info_url': camera_config,  # Camera calibration information
                    },
                ],
                # Remap outputs to the correct namespace
                remappings=[
                    ('/image_raw', '/' + namespace + '/image_raw'),
                    ('/camera_info', '/' + namespace + '/camera_info'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ],
        output='screen',
    )
    
    # argus camera node
    container_argus = ComposableNodeContainer(
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "argus"'])),
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gmsl_ros2',
                plugin='gscam2::GSCamNode',
                name='gmsl_publisher',
                namespace=namespace,
                parameters=[
                    {
                        'gst_config': (['nvarguscamerasrc sensor-id=', index, ' ! video/x-raw(memory:NVMM), format=NV12, width=', width,', height=', height,', framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)UYVY ! videoconvert']),
                        'preroll': False,
                        'use_gst_timestamps': False,
                        'frame_id': frame_id,
                        'camera_name': camera_name,
                        'camera_info_url': camera_config,  # Camera calibration information
                    },
                ],
                # Remap outputs to the correct namespace
                remappings=[
                    ('/image_raw', '/' + namespace + '/image_raw'),
                    ('/camera_info', '/' + namespace + '/camera_info'),
                ],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }],
            ),
        ],
        output='screen',
    )

    # TF publisher
    tf_pub_node = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0.3", "0", "0", "0", "map", frame_id])

    # Rviz2
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'image_view.rviz')],
            condition=IfCondition(open_rviz)
            )

    return LaunchDescription([container_v4l2, container_argus, tf_pub_node, rviz_node])
