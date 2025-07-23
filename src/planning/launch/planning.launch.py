from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # 将所有参数放在一个字典中
    planning_params = {
        'local_topic_name':'/localization/rtk_info',
        'service_name' : '/routing_service',
        'traj_topic_name' : '/planning/trajectory',
        'perc_topic_name' : '/perception/obstacles',
        'left_boundary_topic_name' : '/map/left_boundary',
        'right_boundary_topic_name' : '/map/right_boundary',
        'occupancy_grid_topic_name' : '/occupancy_grid',  # 占用栅格地图话题
        'visualization_topic_name' : '/planning/visualization',  # 可视化话题名
        'remote_control_topic_name' : '/remote_controller/cmd',  # 远程控制话题名
        'remote_control_enabled' : False,  # 是否启用远程控制
        
        'use_occupancy_grid' : True,   # 是否使用占用栅格地图进行障碍物检测
        
        'min_obstacle_distance' : 15.0,  # 最小障碍物距离阈值(米)
        'front_obstacle_width' : 1.0,    # 前方障碍物区域宽度(±米)
        'side_obstacle_width' : 1.0,     # 侧方障碍物区域距离(±米外)
        'occupied_threshold' : 20,        # 占用阈值 (0-100)

        # 方向稳定性参数
        'direction_stability_weight' : 2.0,  # 方向稳定性权重
        'max_index_jump' : 100.0,             # 最大索引跳跃限制
        'yaw_weight' : 3.0,                  # 航向差异权重
        
        'path_type' : 1,         # 路径标号
        'process_frq' : 10.0,    # 处理频率, Hz
        'preview_dist' : 12.0,   # 预览距离, m
        'preview_time' : 1.0,     # 预瞄时间, s
        'start_dist' : 5.0,      # 起始距离, m
        'traj_pub_interval' : 0.1, # 路径发布间隔, 秒
        'planning_spd' : 3.0,     # 规划速度, m/s
        'path_end_dist' : 3.0,    # 路径结束距离, m
        'reverse_moving' : False, # 是否反向行驶
        'test_mode' : False,      # 启用测试模式，生成测试轨迹

        # 性能统计配置
        'enable_timing_logs' : True,         # 是否启用耗时日志输出
        'timing_log_interval' : 10,          # 耗时日志输出间隔（每N帧输出一次）
        'enable_detailed_timing' : True,     # 是否启用详细的分步耗时统计（包含各步骤时间分布）
        'enable_zero_copy' : True,           # 是否启用零拷贝优化（提高栅格地图处理性能）

        # 栅格地图优化配置
        'max_obstacles_to_check' : 30,       # 最大检查的障碍物数量（减少计算量）
        'grid_sampling_resolution' : 0.15,   # 栅格采样分辨率，米（增大以减少计算点数）
        'skip_boundary_check' : False         # 使用完整的边界检查（未优化版本）
    }
    
    # 配置节点，并将参数字典直接传递给参数字段
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',  # 保持与代码中一致
        output='screen',
        parameters=[planning_params],  # 直接使用参数字典
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_log_level,
        planning_node
    ])
