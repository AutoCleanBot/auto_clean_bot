#pragma once

#include <rclcpp/rclcpp.hpp>


namespace routing{
    class RoutingNode : public rclcpp::Node{
        public:
            RoutingNode();
        private:
            void InitParams();
            

    }
}