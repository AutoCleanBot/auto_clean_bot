#pragma once

#include <rclcpp/rclcpp.hpp>
#include <bot_msg/srv/routing.hpp>
#include <unordered_map>
namespace routing{
class RoutingNode : public rclcpp::Node{
    public:
        RoutingNode();
    private:
        void InitParams();
        void HandleRoutingRequest(const bot_msg::srv::Routing::Request::SharedPtr request, bot_msg::srv::Routing::Response::SharedPtr response);
        rclcpp::Service<bot_msg::srv::Routing>::SharedPtr m_service;
        std::vector<std::string> m_map_names;
};

} // namespace routing