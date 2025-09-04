#include "canbus/canbus_node.h"
#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>
#include <stdint.h>
#include <sys/select.h>

namespace canbus {
CanbusNode::CanbusNode() : Node("canbus_node") {
    running_ = true;
    control_cmd_cnt_ = 0;
    mannula_control_flag_ = false;
    InitParams();
    bool ret = InitCanSocket(can_device_name_, can_baudrate_);
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "CAN socket initialization failed");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "CAN socket initialized successfully");

    // 初始化订阅者和发布者
    sub_control_cmd_ = this->create_subscription<bot_msg::msg::ControlCmd>(
        control_cmd_topic_, 10, std::bind(&CanbusNode::ControlCmdCallback, this, std::placeholders::_1));
    sub_remote_controller_ = this->create_subscription<std_msgs::msg::Int32>(
        remote_controller_topic_, 10, std::bind(&CanbusNode::RemoteControllerCallback, this, std::placeholders::_1));
    pub_chassis_info_ = this->create_publisher<bot_msg::msg::ChassisInfo>(chassis_info_topic_, 10);
    // 初始化定时器
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CanbusNode::TimerCallback, this));
    // 创建一个线程循环读取CAN数据
    can_thread_ = std::thread(&CanbusNode::CanThreadFunc, this);
}



CanbusNode::~CanbusNode() {
    running_ = false; // 设置标志位通知线程退出
    if (can_thread_.joinable()) {
        can_thread_.join(); // 等待线程结束
    }
    if (can_fd_ > 0) {
        close(can_fd_);
    }
}

/**
 * @brief 发送给底盘控制的can消息
 *         ! 需要一次性发送两帧消息
 *
 * @param steer_angle
 * @param brk
 * @param gear
 * @param spd
 */
void CanbusNode::SendCtrlMsg(double steer_angle, double brk, uint8_t gear, double spd) {
    can_frame frame;
    frame.can_id = CONTROL_CMD;
    frame.can_dlc = 8;
    FillCanCtrlCmd(frame.data, steer_angle, brk, gear, spd);
    int ret = write(can_fd_, &frame, sizeof(frame));
    if (ret < 0) {
        // RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame");
    }
    PrintCanDataFrame(frame);

    frame.can_id = PERIPH_CMD;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);
    ret = write(can_fd_, &frame, sizeof(frame));
    if (ret < 0) {
        // RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame");
    }
    PrintCanDataFrame(frame);
}

/**
 * @brief 定时器回调函数, 定时上发底盘状态信息;且如无控制消息时,定时下发维持连接的控制消息
 *
 */
void CanbusNode::TimerCallback() {
    auto msg = std::make_shared<bot_msg::msg::ChassisInfo>();
    // RCLCPP_INFO(this->get_logger(), "TimerCallback");
    FillChassisInfo(msg);
    pub_chassis_info_->publish(*msg);

    if (control_cmd_cnt_ > 10 && !mannula_control_flag_) { // 保持无人驾驶的控制连接
        SendCtrlMsg(0.0, 0.0, 0, 0);
    }

    ++control_cmd_cnt_;
    if (control_cmd_cnt_ > 99) {
        control_cmd_cnt_ = 100;
    }
}

/**
 * 填充底盘信息
 */
void CanbusNode::FillChassisInfo(bot_msg::msg::ChassisInfo::SharedPtr msg) {
    msg->steer_angle = chassis_info_local_.steering_wheel_angle;
    msg->brk_press = chassis_info_local_.service_brake_percentage_feedback; // 压力百分比
    msg->cur_speed = chassis_info_local_.speed_feedback;
    msg->soc = chassis_info_local_.soc;
    if (chassis_info_local_.forward_gear_feedback == 1) {
        msg->gear = 1;
        msg->direction = 1;
    } else if (chassis_info_local_.reverse_gear_feedback == 1) {
        msg->gear = 2;
        msg->direction = 2;
    } else {
        msg->gear = 0;
        msg->direction = 0;
    }
    msg->vcu_mode = static_cast<int8_t>(chassis_info_local_.whole_mode);
    msg->controller_online_sts = 0;
    msg->ipc_online_sts = 0;
}

void CanbusNode::InitParams() {
    this->declare_parameter<std::string>("can_device", "can0");
    this->declare_parameter<int>("can_baud", 500);
    this->declare_parameter<std::string>("control_cmd_topic", "/control_cmd");
    this->declare_parameter<std::string>("chassis_info_topic", "/chassis_info_topic");
    this->declare_parameter<std::string>("remote_controller_topic", "/remote_controller");

    this->get_parameter("can_device", can_device_name_);
    this->get_parameter("can_baud", can_baudrate_);
    this->get_parameter("control_cmd_topic", control_cmd_topic_);
    this->get_parameter("chassis_info_topic", chassis_info_topic_);
    this->get_parameter("remote_controller_topic", remote_controller_topic_);

    RCLCPP_INFO(this->get_logger(), "can_device_name: %s", can_device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "can_baud: %d", can_baudrate_);
    RCLCPP_INFO(this->get_logger(), "control_cmd_topic: %s", control_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "chassis_info_topic: %s", chassis_info_topic_.c_str());
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
bool CanbusNode::InitCanSocket(std::string can_device_name, int can_baudrate) {
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

void CanbusNode::CanThreadFunc() {
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
            struct can_frame frame;
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
            if (nbytes < sizeof(struct can_frame)) {
                RCLCPP_WARN(this->get_logger(), "Incomplete CAN frame received, got %d bytes", nbytes);
                continue;
            }
            // 重置空读计数器
            empty_reads_count = 0;
            // 解析CAN数据
            if (frame.can_id == CONTROL_INFO) {
                // 解析CONTROL_INFO
                ControlInfo ctrl_info;
                memcpy(&ctrl_info, frame.data, sizeof(ctrl_info));
                chassis_info_local_.auto_enable = ctrl_info.auto_enable;
                chassis_info_local_.whole_mode = ctrl_info.whole_mode;
                chassis_info_local_.emer_stop_mode = ctrl_info.emer_stop_mode;
                chassis_info_local_.service_brake_status = ctrl_info.service_brake_status;
                chassis_info_local_.parking_brake_status = ctrl_info.parking_brake_status;
                chassis_info_local_.forward_gear_feedback = ctrl_info.forward_gear_feedback;
                chassis_info_local_.reverse_gear_feedback = ctrl_info.reverse_gear_feedback;
                chassis_info_local_.safety_edge_status = ctrl_info.safety_edge_status;

                chassis_info_local_.service_brake_percentage_feedback =
                    static_cast<double>(ctrl_info.service_brake_percentage_feedback) * 0.4 / 100.0;

                chassis_info_local_.speed_feedback =
                    ctrl_info.travel_motor_speed_feedback / 24.2 / 60 * 0.71 * M_PI; // 轮上转速m/s
                chassis_info_local_.steering_wheel_angle = static_cast<double>(ctrl_info.steering_wheel_angle) * 0.01;
                PrintCanDataFrame(frame);
            } else if (frame.can_id == CONTROL_PHY_INFO) {
                // 解析VCU_INFO_2
                ControlPhyInfo ctrl_phy_info;
                memcpy(&ctrl_phy_info, frame.data, sizeof(ctrl_phy_info));
            } else if (frame.can_id == VEHICLE_STATUS_FEEDBACK) {
                // 解析VCU_INFO_DIAG
                VechicleStatusFeedback vechicle_sts;
                memcpy(&vechicle_sts, frame.data, sizeof(vechicle_sts));
            } else if (frame.can_id == BMS_STATUS_FEEDBACK) {
                // 解析VCU_INFO_SPD
                BmsStatusFeedback bms_sts;
                memcpy(&bms_sts, frame.data, sizeof(bms_sts));
                chassis_info_local_.soc = static_cast<double>(bms_sts.soc) * 0.4 / 100.0;
            }
            // PrintCanDataFrame(frame);
        }
    }
    RCLCPP_INFO(this->get_logger(), "CAN thread exiting");
}

/**
 * 控制指令回调函数, 目前的策略是直接转发来自于上层控制器的控制指令
 */
void CanbusNode::ControlCmdCallback(const bot_msg::msg::ControlCmd::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "recv control cmd");
    // 解析控制指令
    double steer_angle = msg->steer_angle * 100.0;
    double brk = msg->brk;
    // double thr = msg->thr;
    uint8_t gear = msg->gear;
    double spd = msg->speed;

    // 发送控制指令
    if(!mannula_control_flag_)
        SendCtrlMsg(steer_angle, brk, gear, spd);
    control_cmd_cnt_ = 0;
}

void CanbusNode::RemoteControllerCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if(msg->data == 3){
        mannula_control_flag_ = false;
    }else if(msg->data == 4){
        mannula_control_flag_ = true;
    }
    RCLCPP_INFO(this->get_logger(), "remote controller key value: %d", msg->data);
}

/**
 * @brief 填充can data frame的控制消息
 *
 * @param data can dataframe 数组值
 * @param steer_angle  方向盘转角, 经过比例放大后
 * @param brk          刹车百分比, 0~1
 * @param gear         档位, 0-N, 1-F, 2-R
 * @param spd          速度, m/s
 */
void CanbusNode::FillCanCtrlCmd(uint8_t data[8], double steer_angle, double brk, uint8_t gear, double spd) {
    // printf("steer_angle: %f, current gear: %d, speed: %f\n", steer_angle, gear, spd);
    // byte0
    memset(data, 0, 8);
    data[0] |= 0x03; // bit0:1 自动模式使能; bit1:1行走使能
    // data[0] &= 0xFB;      // bit2:0 行车制动无效
    data[0] |= 0x01 << 3; // bit3:1 转向使能
    // data[0] &= 0xEF;        // bit4:0 充电使能无效
    if (gear == 1) { // 档位控制
        data[0] |= 1 << 5;
    } else if (gear == 2) {
        data[0] |= 1 << 6;
    }

    // byte1, 刹车百分比控制
    uint8_t target_brk_press = static_cast<uint8_t>(brk * 250);
    data[1] |= target_brk_press; // target_brk_press = target_brk_press; // 目标刹车压力, 0~250

    // byte2~byte3, 方向盘转角控制
    uint16_t target_steer_angle = static_cast<int16_t>(steer_angle);
    data[2] = target_steer_angle & 0xFF;
    data[3] = (target_steer_angle >> 8) & 0xFF;

    // byte4~byte5, 行车转速控制
    int16_t motor_spd = static_cast<int16_t>((spd * 22.4 * 60) / (0.71 * M_PI));
    data[4] = motor_spd & 0xFF;
    data[5] = (motor_spd >> 8) & 0xFF;
    // byte6, 行走电机加速率
    data[6] = 5;
    // byte7, 行走电机减速率
    data[7] = 5;
}

/**
 * @brief 打印can消息
 *
 * @param frame
 */
void CanbusNode::PrintCanDataFrame(const struct can_frame &frame) {
    std::stringstream ss;
    ss << "CAN frame, ID: 0x" << std::hex << frame.can_id << ", Length: " << std::dec << static_cast<int>(frame.can_dlc)
       << ", Data: ";
    for (int i = 0; i < frame.can_dlc; i++) {
        ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

} // namespace canbus

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<canbus::CanbusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}