#include <rclcpp/rclcpp.hpp>
#include <bot_msg/msg/remote_controller.hpp>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <chrono>
#include <functional>

namespace remote_controller_simulator {

class SimpleRemoteControllerSimulator : public rclcpp::Node {
public:
    SimpleRemoteControllerSimulator() : Node("remote_controller_simulator") {
        // 创建发布者
        publisher_ = this->create_publisher<bot_msg::msg::RemoteController>("/remote_controller/cmd", 10);
        
        // 设置终端为无缓冲模式
        setupTerminal();
        
        RCLCPP_INFO(this->get_logger(), "Simple Remote Controller Simulator started");
        printInstructions();
        
        // 创建定时器来检查输入
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 更频繁的检查
            std::bind(&SimpleRemoteControllerSimulator::checkInput, this)
        );
    }

private:
    void setupTerminal() {
        // 设置stdout为无缓冲
        std::cout.setf(std::ios::unitbuf);
        std::cerr.setf(std::ios::unitbuf);
        
        // 强制刷新所有输出流
        std::cout.flush();
        std::cerr.flush();
        fflush(stdout);
        fflush(stderr);
    }

    void printInstructions() {
        std::cout << "\n=== Remote Controller Simulator ===" << std::endl;
        std::cout << "按键功能说明:" << std::endl;
        std::cout << "1 - 起步" << std::endl;
        std::cout << "2 - 停止" << std::endl;
        std::cout << "3 - 自动驾驶" << std::endl;
        std::cout << "4 - 手动接管" << std::endl;
        std::cout << "5 - 任务路径切换" << std::endl;
        std::cout << "6 - 按键6 (扩展功能)" << std::endl;
        std::cout << "7 - 按键7 (扩展功能)" << std::endl;
        std::cout << "8 - 按键8 (扩展功能)" << std::endl;
        std::cout << "q - 退出程序" << std::endl;
        std::cout << "=================================" << std::endl;
        
        // 确保输出立即显示
        std::cout.flush();
        std::cerr.flush();
        
        std::cout << "请输入命令 (1-8 或 q): " << std::flush;
    }

    void checkInput() {
        // 检查是否有输入可用
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        
        int result = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
        
        if (result > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            std::string input;
            std::getline(std::cin, input);
            
            if (!input.empty()) {
                processInput(input[0]);
            }
            
            if (rclcpp::ok()) {
                std::cout << "请输入命令 (1-8 或 q): " << std::flush;
            }
        }
    }
    
    void processInput(char ch) {
        switch (ch) {
            case '1':
                publishKeyCommand(1);
                std::cout << "✓ 发送: 起步命令 (1)" << std::endl;
                break;
            case '2':
                publishKeyCommand(2);
                std::cout << "✓ 发送: 停止命令 (2)" << std::endl;
                break;
            case '3':
                publishKeyCommand(3);
                std::cout << "✓ 发送: 自动驾驶命令 (3)" << std::endl;
                break;
            case '4':
                publishKeyCommand(4);
                std::cout << "✓ 发送: 手动接管命令 (4)" << std::endl;
                break;
            case '5':
                publishKeyCommand(5);
                std::cout << "✓ 发送: 任务路径切换命令 (5)" << std::endl;
                break;
            case '6':
                publishKeyCommand(6);
                std::cout << "✓ 发送: 按键6命令 (6)" << std::endl;
                break;
            case '7':
                publishKeyCommand(7);
                std::cout << "✓ 发送: 按键7命令 (7)" << std::endl;
                break;
            case '8':
                publishKeyCommand(8);
                std::cout << "✓ 发送: 按键8命令 (8)" << std::endl;
                break;
            case 'q':
            case 'Q':
                std::cout << "退出程序..." << std::endl;
                rclcpp::shutdown();
                return;
            default:
                std::cout << "❌ 无效输入: '" << ch << "'" << std::endl;
                std::cout << "请输入1-8或q退出" << std::endl;
                break;
        }
    }

    void publishKeyCommand(int key_num) {
        auto message = bot_msg::msg::RemoteController();
        message.key_value = key_num;
        
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Published key command: %d", key_num);
    }

    rclcpp::Publisher<bot_msg::msg::RemoteController>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace remote_controller_simulator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<remote_controller_simulator::SimpleRemoteControllerSimulator>();
    
    // 使用spin来保持节点运行
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}