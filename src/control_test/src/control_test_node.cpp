#include "control_test/control_test_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto subscriber_node = std::make_shared<LocalizationInfoSubscriber>();
    auto publisher_node = std::make_shared<TrajectoryPublisher>();

    executor.add_node(subscriber_node);
    executor.add_node(publisher_node);

    executor.spin();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}