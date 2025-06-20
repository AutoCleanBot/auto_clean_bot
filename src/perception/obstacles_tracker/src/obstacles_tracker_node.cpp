#include "obstacles_tracker/obstacles_tracker.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<obstacles_tracker::ObstaclesTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 