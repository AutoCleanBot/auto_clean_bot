#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace remote_controller_simulator {

class RemoteControllerSimulator : public rclcpp::Node {
public:
    RemoteControllerSimulator();
    ~RemoteControllerSimulator();

private:
    void keyboardInputThread();
    void publishKeyCommand(int key_num);
    void printInstructions();
    int getch(); // 获取单个字符输入，无需回车
    void restoreTerminal();
    void setupTerminal();

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    std::thread keyboard_thread_;
    struct termios old_termios_;
    bool running_;
};

} // namespace remote_controller_simulator