/**
 * @brief Planning class
 */
#include <rclcpp/rclcpp.hpp>
#include "bot_msg/msg/adc_trajectory.hpp"
#include "bot_msg/msg/localization_info.hpp"
#include "bot_msg/msg/obstacles.hpp"
#include "bot_msg/msg/boundary.hpp"
#include <limits>
namespace planning {

enum PlanningStatus {
    Init = 0,
    Ready = 1,
    Planning = 2,
    Stop = 3,
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
    void UpdatePlanningStatus();
    void UpdateObstacleInfo();
    void FillPubTraj(bot_msg::msg::ADCTrajectory& pub_traj);
    bool IsPathTail();
    void LeftBoundaryCallback(const bot_msg::msg::Boundary::SharedPtr msg);
    void RightBoundaryCallback(const bot_msg::msg::Boundary::SharedPtr msg);
    
    // 新增边界处理相关方法
    bool IsObstacleInBoundary(const auto &obstacle);
    int FindNearestBoundaryPoint(const bot_msg::msg::Boundary &boundary, double east, double north);
    double CalculatePointToBoundaryDistance(double east, double north, double bound_east, double bound_north);
private:
    rclcpp::TimerBase::SharedPtr timer_;
    bot_msg::msg::LocalizationInfo cur_local_;
    bot_msg::msg::ADCTrajectory g_traj_; // 暂时的全局路径
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_; // 订阅localization信息
    rclcpp::Publisher<bot_msg::msg::ADCTrajectory>::SharedPtr pub_traj_; // 发布路径
    rclcpp::Subscription<bot_msg::msg::Obstacles>::SharedPtr sub_perc_; // 订阅感知信息
    rclcpp::Subscription<bot_msg::msg::Boundary>::SharedPtr sub_left_boundary_; // 订阅左边界信息
    rclcpp::Subscription<bot_msg::msg::Boundary>::SharedPtr sub_right_boundary_; // 订阅右边界信息

    std::string local_topic_name_ ; // 定位话题名
    std::string service_name_ ;     // 服务名称
    std::string traj_topic_name_ ;  // 轨迹发布话题名
    std::string perc_topic_name_ ;  // 感知话题名
    std::string left_boundary_topic_name_ ;  // 左边界话题名
    std::string right_boundary_topic_name_ ;  // 右边界话题名
    double process_frq_ ;          // 处理频率
    int path_type_ ;               // 路径类型
    double preview_dist_ ;         // 预览距离
    double start_dist_ ;           // 起始距离
    double traj_pub_interval_ ;    // 路径发布间隔
    double planning_spd_ ;         // 规划速度
    int traj_pub_cnt_ ;            // 路径发布计数
    int timer_cnt_ ;               // 计时器计数
    uint8_t planning_status_ ;     // 规划状态
    double path_end_dist_ ;        // 路径结束距离
    bot_msg::msg::Obstacles obstacles_ ; // 当前感知信息
    bot_msg::msg::Boundary left_boundary_ ; // 左边界信息
    bot_msg::msg::Boundary right_boundary_ ; // 右边界信息
    std::array<int, 3> obstacle_info_ ; // 障碍物信息 0: 左前,1 正前方,2 右前, 存储的是障碍物的标号

    // 临时使用变量
    std::size_t closet_idx_ = 0; // 当前路径下最近点的下标
    bool reverse_moving_ = false; // 是否反向行驶
};
}  // namespace planning
