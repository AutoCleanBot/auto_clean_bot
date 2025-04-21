#include "transform/transform.h"

namespace transform
{
    Transform::Transform() : Node("transform")
    {
        RCLCPP_INFO(this->get_logger(), "Transform node initialized");
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<transform::Transform>());
    rclcpp::shutdown();
    return 0;
}