#include "remote_controller/remote_controller_node.h"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <stdint.h>
#include <sys/select.h>

namespace remote_controller {
RemoteControllerNode::RemoteControllerNode() : Node("remote_controller_node") {
    running_ = true;

    InitParams();
    bool ret = InitCanSocket(can_device_name_, can_baudrate_);
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "CAN socket initialization failed");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "CAN socket initialized successfully");

    pub_remote_controller_ = this->create_publisher<bot_msg::msg::RemoteController>(remote_controller_topic_, 10);

    // 初始化定时器
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&RemoteControllerNode::TimerCallback, this));

    // 创建一个线程循环读取CAN数据
    can_thread_ = std::thread(&RemoteControllerNode::CanThreadFunc, this);
}

RemoteControllerNode::~RemoteControllerNode() {
    running_ = false; // 设置标志位通知线程退出
    if (can_thread_.joinable()) {
        can_thread_.join(); // 等待线程结束
    }
    if (can_fd_ > 0) {
        close(can_fd_);
    }
}

/**
 * @brief 定时器回调函数, 定时发布遥控器消息
 *
 */
void RemoteControllerNode::TimerCallback() {
    auto msg = std::make_shared<bot_msg::msg::RemoteController>();
    // RCLCPP_INFO(this->get_logger(), "TimerCallback");
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = "remote_controller";
    msg->key_value = key_value_;
    pub_remote_controller_->publish(*msg);
}

void RemoteControllerNode::InitParams() {
    this->declare_parameter<std::string>("can_device_name", "can0");
    this->declare_parameter<int>("can_baudrate", 500);
    this->declare_parameter<std::string>("remote_controller_topic", "/remote_controller_topic");

    this->get_parameter("can_device_name", can_device_name_);
    this->get_parameter("can_baudrate", can_baudrate_);
    this->get_parameter("remote_controller_topic", remote_controller_topic_);

    RCLCPP_INFO(this->get_logger(), "can_device_name: %s", can_device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "can_baud: %d", can_baudrate_);
    RCLCPP_INFO(this->get_logger(), "remote_controller_topic: %s", remote_controller_topic_.c_str());
}

/**
 * @brief 初始化can socket
 *
 * @param can_device_name
 * @param can_baudrate
 * @return true
 * @return false
 */
bool RemoteControllerNode::InitCanSocket(std::string can_device_name, int can_baudrate) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 创建SocketCAN套接字
    if ((can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error while opening socket: %s", strerror(errno));
        return false;
    }

    // 设置非阻塞模式
    int flags = fcntl(can_fd_, F_GETFL, 0);
    fcntl(can_fd_, F_SETFL, flags | O_NONBLOCK);
    RCLCPP_INFO(this->get_logger(), "Socket set to non-blocking mode");

    // 指定CAN设备名称
    strcpy(ifr.ifr_name, can_device_name.c_str());
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting interface index for %s: %s", can_device_name.c_str(),
                     strerror(errno));
        close(can_fd_);
        return false;
    }

    // 绑定套接字
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind: %s", strerror(errno));
        close(can_fd_);
        return false;
    }

    // 检查设备是否启动
    if (ioctl(can_fd_, SIOCGIFFLAGS, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting interface flags: %s", strerror(errno));
    } else {
        if (!(ifr.ifr_flags & IFF_UP)) {
            RCLCPP_WARN(this->get_logger(), "CAN interface %s is NOT UP!", can_device_name.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN interface %s is UP and running", can_device_name.c_str());
        }
    }

    RCLCPP_INFO(this->get_logger(), "CAN socket initialized successfully on %s with baudrate %d",
                can_device_name.c_str(), can_baudrate);
    return true;
}

/**
 * 单独的线程循环读取CAN数据
 */

void RemoteControllerNode::CanThreadFunc() {
    RCLCPP_INFO(this->get_logger(), "CAN thread started");
    fd_set rdfs;
    struct timeval tv;
    int activity;
    int empty_reads_count = 0;

    while (running_ && rclcpp::ok()) {
        FD_ZERO(&rdfs);
        FD_SET(can_fd_, &rdfs);

        // 设置超时为100毫秒
        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        // 使用select等待数据可读
        activity = select(can_fd_ + 1, &rdfs, NULL, NULL, &tv);

        if (activity < 0) {
            RCLCPP_ERROR(this->get_logger(), "Select error: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (activity == 0) {
            // 超时，没有数据
            empty_reads_count++;
            if (empty_reads_count % 100 == 0) { // 每10秒左右记录一次
                RCLCPP_WARN(this->get_logger(), "No CAN data received for ~10 seconds");
            }
            continue;
        }

        // 有数据可读
        if (FD_ISSET(can_fd_, &rdfs)) {
            struct canfd_frame frame;
            ssize_t nbytes = read(can_fd_, &frame, sizeof(frame));
            if (nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // 非阻塞模式下暂时没有数据
                    continue;
                }
                RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame: %s", strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            ssize_t frame_size = sizeof(struct canfd_frame);
            if (nbytes < frame_size) {
                RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame received, got %d bytes", nbytes);
                continue;
            }

            if (frame.can_id == 0x0C20122D) {
                auto key_data = frame.data;
                if (key_data[0] == 0x01) {
                    key_value_ = 1;
                }
                else if (key_data[0] == 0x02) {
                    key_value_ = 2;
                }
                else if (key_data[0] == 0x04) {
                    key_value_ = 3;
                }
                else if (key_data[0] == 0x08) {
                    key_value_ = 4;
                }
                else if (key_data[1] == 0x01) {
                    key_value_ = 5;
                }
                else if (key_data[1] == 0x02) {
                    key_value_ = 6;
                }else{
                    key_value_ = 0;
                }
                RCLCPP_INFO(this->get_logger(), "key_value_: %d", key_value_);
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "CAN thread exiting");
}






} // namespace remote_controller

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<remote_controller::RemoteControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}