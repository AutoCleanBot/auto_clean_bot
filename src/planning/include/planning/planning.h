/**
 * @brief Planning class
 */
#include "bot_msg/msg/adc_trajectory.hpp"
#include "bot_msg/msg/boundary.hpp"
#include "bot_msg/msg/localization_info.hpp"
#include "bot_msg/msg/obstacles.hpp"
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace planning {

enum PlanningStatus {
    Init = 0,
    Ready = 1,
    Planning = 2,
    Stop = 3,
    NearPathTail = 4
};

// 障碍物点结构体
struct ObstaclePoint {
    double x;
    double y;
    bool in_boundary;
};

class PlanningNode : public rclcpp::Node {
  public:
    PlanningNode();
    ~PlanningNode();
    void InitGlobalPath();
    void InitParams();

    void TimerCallback();
    void LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
    void ObstaclesCallback(const bot_msg::msg::Obstacles::SharedPtr msg);
    void OccupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void RemoteControlCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void UpdatePlanningStatus();
    void UpdateObstacleInfo();
    void UpdateObstacleInfoFromOccupancyGrid();
    void FillPubTraj(bot_msg::msg::ADCTrajectory &pub_traj);
    bool IsNearDistance(const double& distance);
    double CalculateDistanceToEnd();

    // 优化的最近点搜索辅助函数
    double calculateTrajectoryPointCost(size_t index, double cur_yaw_rad, size_t last_closest_idx, bool first_run);
    void LeftBoundaryCallback(const bot_msg::msg::Boundary::SharedPtr msg);
    void RightBoundaryCallback(const bot_msg::msg::Boundary::SharedPtr msg);
    bool IsObstacleInBoundaryByPosition(double global_x, double global_y);

    // 可视化相关函数
    void PublishVisualization(const bot_msg::msg::ADCTrajectory &pub_traj);
    visualization_msgs::msg::Marker CreateTrajectoryMarker(const bot_msg::msg::ADCTrajectory &pub_traj);
    visualization_msgs::msg::Marker CreateVehicleMarker();
    visualization_msgs::msg::Marker CreateObstacleStatusMarker();
    visualization_msgs::msg::Marker CreateCoordinateMarker();
    visualization_msgs::msg::Marker CreateBoundaryMarker(const bot_msg::msg::Boundary &boundary, const std::string &ns,
                                                         const std_msgs::msg::ColorRGBA &color);

    // 新增边界处理相关方法
    bool IsObstacleInBoundary(const bot_msg::msg::ObstacleInfo &obstacle);
    int FindNearestBoundaryPoint(const bot_msg::msg::Boundary &boundary, double east, double north);
    double CalculatePointToBoundaryDistance(double east, double north, double bound_east, double bound_north);

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    bot_msg::msg::LocalizationInfo cur_local_;
    bot_msg::msg::ADCTrajectory g_traj_;
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_; // 订阅localization信息
    rclcpp::Publisher<bot_msg::msg::ADCTrajectory>::SharedPtr pub_traj_;                    // 发布路径
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_visualization_;  // 发布可视化信息
    rclcpp::Subscription<bot_msg::msg::Obstacles>::SharedPtr sub_perc_;                     // 订阅感知信息
    rclcpp::Subscription<bot_msg::msg::Boundary>::SharedPtr sub_left_boundary_;             // 订阅左边界信息
    rclcpp::Subscription<bot_msg::msg::Boundary>::SharedPtr sub_right_boundary_;            // 订阅右边界信息
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;      // 订阅占用栅格地图
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_remote_control_;              // 订阅远程控制信息

    std::string local_topic_name_;          // 定位话题名
    std::string service_name_;              // 服务名称
    std::string traj_topic_name_;           // 轨迹发布话题名
    std::string perc_topic_name_;           // 感知话题名
    std::string left_boundary_topic_name_;  // 左边界话题名
    std::string right_boundary_topic_name_; // 右边界话题名
    std::string occupancy_grid_topic_name_; // 占用栅格地图话题名
    std::string visualization_topic_name_;  // 可视化话题名
    std::string remote_control_topic_name_; // 远程控制话题名

    double process_frq_;                          // 处理频率
    int path_type_;                               // 路径类型
    double preview_dist_;                         // 预览距离
    double start_dist_;                           // 起始距离
    double traj_pub_interval_;                    // 路径发布间隔
    double planning_spd_;                         // 规划速度
    int traj_pub_cnt_;                            // 路径发布计数
    int timer_cnt_;                               // 计时器计数
    uint8_t planning_status_;                     // 规划状态
    double path_end_dist_;                        // 路径结束距离
    bot_msg::msg::Obstacles obstacles_;           // 当前感知信息
    bot_msg::msg::Boundary left_boundary_;        // 左边界信息
    bot_msg::msg::Boundary right_boundary_;       // 右边界信息
    nav_msgs::msg::OccupancyGrid occupancy_grid_; // 当前占用栅格地图
    std::array<int, 3> obstacle_info_;            // 障碍物信息 0: 左前,1 正前方,2 右前, 存储的是障碍物的标号
    bool use_occupancy_grid_;                     // 是否使用占用栅格地图进行障碍物检测
    bool test_mode_;                              // 测试模式，不依赖routing服务
    bool remote_control_enabled_;                 // 是否启用远程控制, 启动远程遥控,则等待遥控才能启动
    bool key_stop_;                               // key_stop_
    bool manula_control_;                         // 手动控制符号位
    int32_t remote_control_cmd_;                  // 远程控制命令

    // 占用栅格地图障碍物检测参数
    double min_obstacle_distance_; // 最小障碍物距离阈值
    double front_obstacle_width_;  // 前方障碍物区域宽度(±米)
    double side_obstacle_width_;   // 侧方障碍物区域距离(±米外)
    int occupied_threshold_;       // 占用阈值 (0-100)

    // 方向稳定性参数
    double direction_stability_weight_; // 方向稳定性权重
    double max_index_jump_;             // 最大索引跳跃限制

    // 临时使用变量
    std::size_t closet_idx_ = 0;  // 当前路径下最近点的下标
    bool reverse_moving_ = false; // 是否反向行驶

    // 存储检测到的障碍物点
    std::vector<ObstaclePoint> detected_obstacle_points_;

    // 性能统计参数
    bool enable_timing_logs_;     // 是否启用耗时日志
    int timing_log_interval_;     // 耗时日志输出间隔（每N帧输出一次）
    bool enable_detailed_timing_; // 是否启用详细的分步耗时统计
    bool enable_zero_copy_;       // 是否启用零拷贝优化

    // 栅格地图优化参数
    int max_obstacles_to_check_;      // 最大检查的障碍物数量
    double grid_sampling_resolution_; // 栅格采样分辨率（米）
    bool skip_boundary_check_;        // 是否跳过复杂的边界检查

    // 终点减速参数
    bool enable_end_deceleration_;    // 是否启用终点前减速
    double deceleration_distance_;    // 开始减速的距离（米）
    double deceleration_speed_;       // 减速后的目标速度（m/s）
    double final_stop_distance_;      // 最终停车距离（米）

    // 性能统计变量
    mutable int frame_count_;                       // 处理的帧数计数
    mutable double total_processing_time_;          // 总处理时间（毫秒）
    mutable double total_obstacle_detection_time_;  // 障碍物检测总时间（毫秒）
    mutable double total_trajectory_planning_time_; // 轨迹规划总时间（毫秒）
    mutable double total_visualization_time_;       // 可视化总时间（毫秒）
    mutable double total_occupancy_grid_time_;      // 占用栅格地图处理总时间（毫秒）

    // 性能统计辅助函数
    void logTimingStatistics() const;
    void resetTimingStatistics() const;
    double getCurrentTimeMs() const;

    // 零拷贝优化函数
    void updateObstacleInfoFromOccupancyGridZeroCopy();

    // 创建障碍物点可视化标记
    visualization_msgs::msg::Marker CreateObstaclePointsMarker();
};
} // namespace planning
