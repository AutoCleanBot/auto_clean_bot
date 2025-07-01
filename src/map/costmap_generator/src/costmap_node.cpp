#include "costmap_generator/costmap_generator.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<costmap_generator::CostmapGenerator>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
