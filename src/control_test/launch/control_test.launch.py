import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # 启动 subscriber_node
        Node(
            package='control_test',
            executable='control_test_node',
            name='subscriber_node',
            output='screen',
            parameters=[{'is_subscriber': True}]  # 可以传递参数来区分这个节点
        ),
        # 启动 publisher_node
        Node(
            package='control_test',
            executable='control_test_node',
            name='publisher_node',
            output='screen',
            parameters=[{'is_subscriber': False}]  # 通过参数来区分
        ),
    ])