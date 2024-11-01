"""Launch a Node with parameters and remappings."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # 设置参数的命名空间和节点名称
    namespace = LaunchConfiguration('namespace', default='camera')  # 默认命名空间为 "camera"
    node_name = LaunchConfiguration('node_name', default='gmsl_camera_node')  # 默认节点名称为 "gmsl_camera_node"
    frame_id = LaunchConfiguration('frame_id', default='gmsl_camera_frame')  # 默认帧 ID 为 "gmsl_camera_frame"
    camera_name = LaunchConfiguration('camera_name', default='gmsl_camera')  # 默认相机名称为 "gmsl_camera"

    # 设置相机分辨率
    width = LaunchConfiguration('width', default='1920')  # 默认宽度为 1920
    height = LaunchConfiguration('height', default='1080')  # 默认高度为 1080

    # 设置相机配置文件的路径
    camera_config = LaunchConfiguration('camera_config', default=[
        TextSubstitution(text='file://' + os.path.join(get_package_share_directory('gmsl_ros2'), 'cfg', '')),  # 获取 gmsl_ros2 包的 cfg 目录
        TextSubstitution(text='calibration_param_example_'),  # 校准参数文件名的前缀
        width,  # 相机宽度
        TextSubstitution(text='x'),  # 硬编码字符 'x'，可用于构建参数
        height,  # 相机高度
        TextSubstitution(text='.yaml')  # 硬编码扩展名 '.yaml'
    ])

    # 配置相机设备和类型
    camera_dev = LaunchConfiguration('camera_dev', default='/dev/video0')  # 默认相机设备为 /dev/video0
    camera_type = LaunchConfiguration('camera_type', default='argus')  # 默认相机类型为 "argus"
    index = LaunchConfiguration('index', default='0')  # 默认索引为 0

    # 创建 V4L2 相机节点
    v4l2_camera_node = Node(
        package='gmsl_ros2',  # 包名称
        executable='gmsl_main',  # 可执行文件名
        output='screen',  # 输出到控制台
        name=node_name,  # 节点名称
        namespace=namespace,  # 命名空间
        parameters=[{  # 设置节点参数
            'gst_config': (['v4l2src device=', camera_dev, ' ! video/x-raw,format=(string)UYVY,framerate=30/1,width=', width, ',height=', height, ' ! videoconvert ! video/x-raw, format=(string)BGR ! videoconvert']),  # GStreamer 管道配置
            'preroll': False,  # 是否启用预滚动
            'use_gst_timestamps': False,  # 是否使用 GStreamer 时间戳
            'frame_id': frame_id,  # 帧 ID
            'camera_name': camera_name,  # 相机名称
            'camera_info_url': camera_config,  # 相机校准信息
        }],
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "v4l2"']))  # 仅在相机类型为 v4l2 时启动
    )

    # 创建网络摄像头节点
    webcam_camera_node = Node(
        package='gmsl_ros2',
        executable='gmsl_main',
        output='screen',
        name=node_name,
        namespace=namespace,
        parameters=[{
            'gst_config': (['v4l2src device=', camera_dev, ' ! videoconvert']),  # GStreamer 管道配置
            'preroll': False,
            'use_gst_timestamps': False,
            'frame_id': frame_id,
            'camera_name': camera_name,
            'camera_info_url': camera_config,  # 相机校准信息
        }],
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "webcam"']))  # 仅在相机类型为 webcam 时启动
    )

    # 创建 Argus 相机节点
    argus_camera_node = Node(
        package='gmsl_ros2',
        executable='gmsl_main',
        output='screen',
        name=node_name,
        namespace=namespace,
        parameters=[{
            'gst_config': (['nvarguscamerasrc sensor-id=', index, ' ! video/x-raw(memory:NVMM), format=NV12, width=', width, ', height=', height, ', framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)UYVY ! videoconvert']),  # Argus GStreamer 管道配置
            'preroll': False,
            'use_gst_timestamps': False,
            'frame_id': frame_id,
            'camera_name': camera_name,
            'camera_info_url': camera_config,  # 相机校准信息
        }],
        condition=IfCondition(PythonExpression(['"', camera_type, '" == "argus"']))  # 仅在相机类型为 argus 时启动
    )

    # 创建 TF 发布器节点
    publish_tf = LaunchConfiguration('publish_tf', default='true')  # 默认启用 TF 发布
    tf_pub_node = Node(package="tf2_ros",
                       executable="static_transform_publisher",
                       arguments=["0", "0", "0.3", "0", "0", "0", "map", frame_id],  # 发布静态变换
                       condition=IfCondition(publish_tf)  # 仅在 publish_tf 为 true 时启动
                       )

    # 启动 RViz2 可视化工具
    open_rviz = LaunchConfiguration('open_rviz', default='false')  # 默认不打开 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(get_package_share_directory('gmsl_ros2'), 'rviz', 'image_view.rviz')],  # 指定 RViz 配置文件
        condition=IfCondition(open_rviz)  # 仅在 open_rviz 为 true 时启动
    )

    return LaunchDescription([v4l2_camera_node, 
                              webcam_camera_node,
                              argus_camera_node,
                              tf_pub_node,
                              rviz_node])  # 返回所有节点的启动描述
