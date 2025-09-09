#pragma once

#include <bot_msg/msg/boundary.hpp>
#include <bot_msg/msg/boundary_point.hpp>
#include <bot_msg/msg/localization_info.hpp>
#include <bot_msg/msg/remote_controller.hpp>
#include <std_msgs/msg/int32.hpp>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace map {

struct BoundaryPoint {
    double east;
    double north;
    double yaw;
};

class MapNode : public rclcpp::Node {
  public:
    MapNode();
    ~MapNode();

  private:
    // 订阅定位信息的回调函数
    void localizationCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
    
    // 订阅遥控器按键的回调函数
    void remoteControllerCallback(const std_msgs::msg::Int32::SharedPtr msg);

    // 定时器回调函数，用于发布边界点
    void timerCallback();

    // 加载边界文件
    bool loadBoundaryFile(const std::string &file_path, std::vector<BoundaryPoint> &boundary_points);
    
    // 根据boundary_type构建边界文件路径
    std::pair<std::string, std::string> getBoundaryFilePaths(int boundary_type);
    
    // 重新加载边界文件
    void reloadBoundaryFiles(int new_boundary_type);

    // 查找最近点索引
    size_t findClosestPointIndex(const std::vector<BoundaryPoint> &boundary_points, double east, double north);

    // 计算单个点的成本（用于优化搜索）
    double calculatePointCost(const std::vector<BoundaryPoint> &boundary_points, size_t index, double cur_yaw_rad,
                              size_t last_closest_idx, bool first_run);

    // 计算指定长度的边界点
    void calculateBoundarySegment(const std::vector<BoundaryPoint> &boundary_points, size_t start_index, double length,
                                  std::vector<bot_msg::msg::BoundaryPoint> &segment_points);
                                  
    // 循环模式辅助函数
    bool IsNearDistance(double distance);  // 判断是否接近边界终点
    double CalculateDistanceToEnd();       // 计算到边界终点的距离

  private:
    // ROS参数
    std::string left_boundary_file_path_;
    std::string right_boundary_file_path_;
    std::string left_boundary_name_;
    std::string right_boundary_name_;
    double boundary_length_;
    double publish_frequency_;
    std::string map_files_dir_;

    // 订阅和发布
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr localization_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr remote_controller_sub_;
    rclcpp::Publisher<bot_msg::msg::Boundary>::SharedPtr left_boundary_pub_;
    rclcpp::Publisher<bot_msg::msg::Boundary>::SharedPtr right_boundary_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 边界点数据
    std::vector<BoundaryPoint> left_boundary_points_;
    std::vector<BoundaryPoint> right_boundary_points_;

    // 当前位置
    double current_east_ = 0.0;
    double current_north_ = 0.0;
    double current_yaw_ = 0.0;
    bool localization_received_ = false;
    
    // 当前边界类型
    int current_boundary_type_ = 3;
    int bkpoint_start_path_type_;
    int bkpoint_end_path_type_;
    int cyclic_test_path_type_;  // 循环测试边界类型
    
    // 循环测试模式相关变量
    bool cyclic_test_mode_ = false;        // 是否处于循环测试模式
    bool cyclic_change_idx_flag_ = false;  // 循环测试模式下的索引跳转标志
    double cyclic_test_end_dis_ = 2.0;     // 循环测试模式下的终点距离

    // 方向稳定性参数
    double direction_stability_weight_ = 2.0; // 方向稳定性权重
    double max_index_jump_ = 30.0;            // 最大索引跳跃限制（边界线通常更密集）
    double yaw_weight_ = 3.0;                 // 航向差异权重
};

} // namespace map