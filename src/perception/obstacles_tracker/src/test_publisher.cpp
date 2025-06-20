#include <rclcpp/rclcpp.hpp>
#include <bot_msg/msg/obstacles.hpp>
#include <bot_msg/msg/obstacle_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <random>

/**
 * @brief 测试发布节点，用于发布模拟的障碍物数据
 */
class TestPublisherNode : public rclcpp::Node {
public:
    TestPublisherNode() : Node("test_publisher_node") {
        // 创建障碍物发布者
        obstacles_pub_ = this->create_publisher<bot_msg::msg::Obstacles>("/perception/obstacles", 10);
        
        // 创建可视化标记发布者
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/obstacles_markers", 10);
        
        // 创建定时器，每100毫秒发布一次数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestPublisherNode::timer_callback, this));
        
        // 初始化随机数生成器
        gen_ = std::mt19937(rd_());
        pos_noise_ = std::normal_distribution<double>(0.0, 0.1);
        size_noise_ = std::normal_distribution<double>(0.0, 0.05);
        
        RCLCPP_INFO(this->get_logger(), "测试发布节点已启动");
    }

private:
    void timer_callback() {
        auto obstacles_msg = std::make_unique<bot_msg::msg::Obstacles>();
        obstacles_msg->header.stamp = this->now();
        obstacles_msg->header.frame_id = "base_link";
        
        // 添加一些模拟的静态障碍物
        addStaticObstacle(*obstacles_msg, 5.0, 5.0, 0.0, 1.0, 1.0, 1.0, 1);
        addStaticObstacle(*obstacles_msg, -5.0, 5.0, 0.0, 0.8, 0.8, 1.2, 2);
        addStaticObstacle(*obstacles_msg, 0.0, 8.0, 0.0, 2.0, 0.5, 0.5, 3);
        
        // 添加移动障碍物
        addMovingObstacle(*obstacles_msg);
        
        // 发布障碍物消息
        obstacles_pub_->publish(std::move(obstacles_msg));
        
        // 发布可视化标记
        visualization_msgs::msg::MarkerArray markers = createMarkers(*obstacles_msg);
        marker_pub_->publish(markers);
    }
    
    void addStaticObstacle(
        bot_msg::msg::Obstacles& obstacles_msg,
        double x, double y, double z,
        double length, double width, double height,
        int type) {
        
        bot_msg::msg::ObstacleInfo obstacle;
        obstacle.id = static_obstacles_count_++;
        obstacle.type = type;
        obstacle.status = 1;
        
        // 添加一些随机噪声，模拟检测误差
        obstacle.position_x = x + pos_noise_(gen_);
        obstacle.position_y = y + pos_noise_(gen_);
        obstacle.position_z = z;
        obstacle.velocity_x = 0.0;
        obstacle.velocity_y = 0.0;
        obstacle.length = length + size_noise_(gen_);
        obstacle.width = width + size_noise_(gen_);
        obstacle.height = height + size_noise_(gen_);
        obstacle.heading = 0.0;
        
        obstacles_msg.obstacles.push_back(obstacle);
    }
    
    void addMovingObstacle(bot_msg::msg::Obstacles& obstacles_msg) {
        // 更新移动障碍物的位置
        moving_obstacle_x_ += 0.1;
        if (moving_obstacle_x_ > 10.0) {
            moving_obstacle_x_ = -10.0;
        }
        
        bot_msg::msg::ObstacleInfo obstacle;
        obstacle.id = 9999;  // 固定ID
        obstacle.type = 4;   // 移动物体
        obstacle.status = 1;
        
        obstacle.position_x = moving_obstacle_x_ + pos_noise_(gen_);
        obstacle.position_y = 0.0 + pos_noise_(gen_);
        obstacle.position_z = 0.0;
        obstacle.velocity_x = 1.0;
        obstacle.velocity_y = 0.0;
        obstacle.length = 1.5 + size_noise_(gen_);
        obstacle.width = 0.8 + size_noise_(gen_);
        obstacle.height = 1.0;
        obstacle.heading = 0.0;
        
        obstacles_msg.obstacles.push_back(obstacle);
    }
    
    visualization_msgs::msg::MarkerArray createMarkers(const bot_msg::msg::Obstacles& obstacles_msg) {
        visualization_msgs::msg::MarkerArray markers;
        
        for (size_t i = 0; i < obstacles_msg.obstacles.size(); ++i) {
            const auto& obstacle = obstacles_msg.obstacles[i];
            
            // 创建边界框标记
            visualization_msgs::msg::Marker box_marker;
            box_marker.header.frame_id = obstacles_msg.header.frame_id;
            box_marker.header.stamp = obstacles_msg.header.stamp;
            box_marker.ns = "raw_obstacle_boxes";
            box_marker.id = obstacle.id;
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 设置位置
            box_marker.pose.position.x = obstacle.position_x;
            box_marker.pose.position.y = obstacle.position_y;
            box_marker.pose.position.z = obstacle.position_z;
            
            // 设置朝向
            tf2::Quaternion q;
            q.setRPY(0, 0, obstacle.heading);
            box_marker.pose.orientation.x = q.x();
            box_marker.pose.orientation.y = q.y();
            box_marker.pose.orientation.z = q.z();
            box_marker.pose.orientation.w = q.w();
            
            // 设置尺寸
            box_marker.scale.x = obstacle.length;
            box_marker.scale.y = obstacle.width;
            box_marker.scale.z = obstacle.height;
            
            // 根据障碍物类型设置颜色
            box_marker.color.r = 1.0;
            box_marker.color.g = 0.0;
            box_marker.color.b = 0.0;
            box_marker.color.a = 0.3;
            
            // 设置生命周期
            box_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            markers.markers.push_back(box_marker);
            
            // 创建ID文本标记
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = box_marker.header;
            text_marker.ns = "raw_obstacle_ids";
            text_marker.id = obstacle.id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 设置位置，略高于障碍物
            text_marker.pose.position.x = obstacle.position_x;
            text_marker.pose.position.y = obstacle.position_y;
            text_marker.pose.position.z = obstacle.position_z + obstacle.height + 0.5;
            
            // 设置文本
            text_marker.text = "ID: " + std::to_string(obstacle.id);
            
            // 设置尺寸
            text_marker.scale.z = 0.5; // 文本高度
            
            // 设置颜色
            text_marker.color.r = 1.0;
            text_marker.color.g = 0.0;
            text_marker.color.b = 0.0;
            text_marker.color.a = 1.0;
            
            // 设置生命周期
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            markers.markers.push_back(text_marker);
        }
        
        return markers;
    }
    
    rclcpp::Publisher<bot_msg::msg::Obstacles>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> pos_noise_;
    std::normal_distribution<double> size_noise_;
    
    double moving_obstacle_x_ = -10.0;
    uint32_t static_obstacles_count_ = 1000;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 