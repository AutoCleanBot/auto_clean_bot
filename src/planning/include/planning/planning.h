/**
 * @brief Planning class
 */
#include <rclcpp/rclcpp.hpp>
#include "bot_msg/msg/adc_trajectory.hpp"
#include "bot_msg/msg/localization_info.hpp"

namespace planning {
class PlanningNode : public rclcpp::Node {
public:
    PlanningNode();
    ~PlanningNode();
    void InitGlobalPath();
    void InitParams();

    void TimerCallback();
    void LocalizationInfoCallback(const bot_msg::msg::LocalizationInfo::SharedPtr msg);
private:
    rclcpp::TimerBase::SharedPtr timer_;
    bot_msg::msg::LocalizationInfo cur_local_;
    bot_msg::msg::ADCTrajectory g_traj_; // 暂时的全局路径
    rclcpp::Subscription<bot_msg::msg::LocalizationInfo>::SharedPtr sub_localization_info_; // 订阅localization信息
    rclcpp::Publisher<bot_msg::msg::ADCTrajectory>::SharedPtr pub_traj_; // 发布路径

    std::string local_topic_name_ ; // 定位话题名
    std::string service_name_ ;     // 服务名称
    std::string traj_topic_name_ ;  // 轨迹发布话题名
    double process_frq_ ;          // 处理频率
    int path_type_ ;               // 路径类型
    double preview_dist_ ;         // 预览距离
    double start_dist_ ;           // 起始距离
    double traj_pub_interval_ ;    // 路径发布间隔
    int traj_pub_cnt_ ;            // 路径发布计数
    int timer_cnt_ ;               // 计时器计数
};
}  // namespace planning
