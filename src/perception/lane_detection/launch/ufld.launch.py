from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('lane_detection')
    
    engine_path = LaunchConfiguration('engine_path')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'engine_path',
            default_value=os.path.join(pkg_dir, 'models', 'ufld_v2.engine'),
            description='Path to the TensorRT engine file'
        ),
        
        Node(
            package='lane_detection',
            executable='ufld_node',
            name='ufld_node',
            parameters=[{
                'engine_path': engine_path,
                'cut_height': 320,
                # 'input_width': 800,
                # 'input_height': 320,
                'input_width': 1920,
                'input_height': 1080,
                'num_row': 72,
                'num_col': 81
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('lane_detection', '/lane_detection/image')
            ]
        )
    ])