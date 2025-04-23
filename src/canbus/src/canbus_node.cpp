#include "canbus/canbus_node.h"
#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <stdint.h>
#include <sys/select.h>

namespace canbus {
CanbusNode::CanbusNode() : Node("canbus_node") {
    running_ = true;
    line_control_ready_ = false;
    motor_en_cnt_ = 0;
    control_cmd_cnt_ = 0;
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

void CanbusNode::TimerCallback() {
    auto msg = std::make_shared<bot_msg::msg::ChassisInfo>();
    // RCLCPP_INFO(this->get_logger(), "TimerCallback");
    FillChassisInfo(msg);
    pub_chassis_info_->publish(*msg);
    ++control_cmd_cnt_;
    if (control_cmd_cnt_ > 99) {
        motor_en_cnt_ = 0;
        RCLCPP_INFO(this->get_logger(), "control_cmd_cnt_: %d", control_cmd_cnt_);
        control_cmd_cnt_ = 100;
    }
}

/**
 * 填充底盘信息
 */
void CanbusNode::FillChassisInfo(bot_msg::msg::ChassisInfo::SharedPtr msg) {
    msg->steer_angle = (static_cast<float>(chassis_info_local_.cur_steer_angle) * 0.1f - 3000.0f);
    msg->brk_press = static_cast<float>(chassis_info_local_.cur_brk_press) * 0.05f / 8.0f; // 压力百分比
    msg->cur_speed = static_cast<float>(chassis_info_local_.current_speed) / 10.0f;
    msg->soc = static_cast<float>(chassis_info_local_.soc) * 0.4f;
    msg->gear = static_cast<int8_t>(chassis_info_local_.cur_gear);
    msg->direction = static_cast<int8_t>(chassis_info_local_.current_direction);
    msg->vcu_mode = static_cast<int8_t>(chassis_info_local_.vcu_mode);
    msg->controller_online_sts = static_cast<int8_t>(chassis_info_local_.controller_online_sts);
    msg->ipc_online_sts = static_cast<int8_t>(chassis_info_local_.ipc_online_sts);
}

void CanbusNode::InitParams() {
    this->declare_parameter<std::string>("can_device", "can0");
    this->declare_parameter<int>("can_baud", 500);
    this->declare_parameter<std::string>("control_cmd_topic", "/control_cmd");
    this->declare_parameter<std::string>("chassis_info_topic", "/chassis_info_topic");

    this->get_parameter("can_device", can_device_name_);
    this->get_parameter("can_baud", can_baudrate_);
    this->get_parameter("control_cmd_topic", control_cmd_topic_);
    this->get_parameter("chassis_info_topic", chassis_info_topic_);

    RCLCPP_INFO(this->get_logger(), "can_device_name: %s", can_device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "can_baud: %d", can_baudrate_);
    RCLCPP_INFO(this->get_logger(), "control_cmd_topic: %s", control_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "chassis_info_topic: %s", chassis_info_topic_.c_str());
}

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

            if (frame.can_id == VCU_INFO_1) {
                PrintCanDataFrame(frame);
            }
            if (frame.can_id == VCU_INFO_2) {
                PrintCanDataFrame(frame);
            }

            // 解析CAN数据
            if (frame.can_id == VCU_INFO_1) {
                // 解析VCU_INFO_1
                CanVCUInfo1 vcu_info_1;
                memcpy(&vcu_info_1, frame.data, sizeof(vcu_info_1));
                chassis_info_local_.ses_en_sts = vcu_info_1.ses_en_sts;
                chassis_info_local_.moter_en_sts = vcu_info_1.moter_en_sts;
                chassis_info_local_.motor_torque = vcu_info_1.motor_torque;
                chassis_info_local_.vcu_mode = vcu_info_1.vcu_mode;
                if (!line_control_ready_ && chassis_info_local_.vcu_mode == 4) {
                    line_control_ready_ = true;
                    RCLCPP_INFO(this->get_logger(), "Line control ready");
                }
                chassis_info_local_.rseb_mode = vcu_info_1.rseb_mode;
                chassis_info_local_.bat_num = vcu_info_1.bat_num;
                chassis_info_local_.cur_gear = vcu_info_1.cur_gear;
                chassis_info_local_.bat_warn_sts = vcu_info_1.bat_warn_sts;
                chassis_info_local_.cur_spd_mode = vcu_info_1.cur_spd_mode;
                chassis_info_local_.controller_online_sts = vcu_info_1.controller_online_sts;
                chassis_info_local_.ipc_online_sts = vcu_info_1.ipc_online_sts;
            } else if (frame.can_id == VCU_INFO_2) {
                // 解析VCU_INFO_2
                CanVCUInfo2 vcu_info_2;
                memcpy(&vcu_info_2, frame.data, sizeof(vcu_info_2));
                chassis_info_local_.soc = frame.data[0];
                chassis_info_local_.park_st = vcu_info_2.park_st;
                chassis_info_local_.coll_sts = vcu_info_2.coll_sts;
                chassis_info_local_.bms_sts = vcu_info_2.bms_sts;
                chassis_info_local_.motor_dir = vcu_info_2.motor_dir;
                chassis_info_local_.auto_mode = vcu_info_2.auto_mode;
                chassis_info_local_.auto_mode_en = vcu_info_2.auto_mode_en;
            } else if (frame.can_id == VCU_INFO_DIAG) {
                // 解析VCU_INFO_DIAG
                CanVCUInfoDiag vcu_info_diag;
                memcpy(&vcu_info_diag, frame.data, sizeof(vcu_info_diag));
                chassis_info_local_.error_level = vcu_info_diag.error_level;
                chassis_info_local_.error_eps_level = vcu_info_diag.error_eps_level;
                chassis_info_local_.error_eb_level = vcu_info_diag.error_eb_level;
                chassis_info_local_.error_bms_level = vcu_info_diag.error_bms_level;
                chassis_info_local_.error_motor_level = vcu_info_diag.error_motor_level;
                chassis_info_local_.ode_info = vcu_info_diag.ode_info;
            } else if (frame.can_id == VCU_INFO_SPD) {
                // 解析VCU_INFO_SPD
                CanVCUInfoSpd vcu_info_spd;
                memcpy(&vcu_info_spd, frame.data, sizeof(vcu_info_spd));
                chassis_info_local_.current_speed = (frame.data[0] & 0xFE) >> 1; // 第一个bit不使用
                chassis_info_local_.current_direction = vcu_info_spd.current_direction;
                chassis_info_local_.current_gear = vcu_info_spd.current_gear;
            } else if (frame.can_id == SEB_INFO) {
                // 解析SEB_INFO
                CanSEBInfo seb_info;
                memcpy(&seb_info, frame.data, sizeof(seb_info));
                chassis_info_local_.cur_brk_press = seb_info.cur_brk_press;
            } else if (frame.can_id == SES_INFO) {
                // 右正左负
                uint16_t raw_angle = (frame.data[1] << 8) | frame.data[2];
                // 存储原始值
                chassis_info_local_.cur_steer_angle = raw_angle;
            } else {
                // RCLCPP_WARN(this->get_logger(), "Unknown CAN frame id: %d", frame.can_id);
            }
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
    double steer_angle = msg->steer_angle;
    double brk = msg->brk;
    // double thr = msg->thr;
    uint8_t gear = msg->gear;
    double spd = msg->speed;

    // 仅当处于线控模式或预备切换模式时，才发送控制指令
    // 注意当遥控器接管后,需要关闭遥控器,才能使车辆回到 mode 4.
    if (chassis_info_local_.vcu_mode == 4 || chassis_info_local_.vcu_mode == 2) {
        // 发送控制指令
        can_frame frame;
        frame.can_id = CONTROL_CMD;
        frame.can_dlc = 8;
        FillCanCtrlCmd(frame.data, steer_angle, brk, gear, spd);
        PrintCanDataFrame(frame);
        int ret = write(can_fd_, &frame, sizeof(frame));

        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame");
        }
        control_cmd_cnt_ = 0;
    }
}
/**
 * 填充控制CAN结构体
 */
void CanbusNode::FillCanCtrlCmd(uint8_t data[8], double steer_angle, double brk, uint8_t gear, double spd) {

    static uint8_t life_signal = 0;
    // printf("steer_angle: %f, current gear: %d, speed: %f\n", steer_angle, gear, spd);
    // byte0
    memset(data, 0, 8);
    data[0] |= 0x01;      // motor_enable = 1;   // 1: 启用
    data[0] |= gear << 1; // target_gear = gear; // 目标档位, 0: N挡, 1: D档, 2: R档
    if (motor_en_cnt_ < 10) {
        data[0] |= 0 << 3; // target_mode = 0;    // 0: 手动模式
        motor_en_cnt_++;
    } else {
        data[0] |= 1 << 3; // target_mode = 1;    // 1: 线控模式
    }

    data[0] |= life_signal++ << 4; // life_signal = life_signal++;
    if (life_signal > 15) {
        life_signal = 0;
    }
    // byte1
    uint8_t target_speed = (spd * 10 > 127) ? 127 : spd * 10;
    data[1] |= target_speed; // target_speed = target_speed; // 目标速度, 0~127, 单位0.1m/s
    data[1] |= 1 << 7;       // target_spd_val = 1;          // 目标速度值有效位, 1: 有效

    // byte2
    uint8_t target_brk_press = static_cast<uint8_t>(brk * 8.0 * 20.0);
    data[2] |= target_brk_press; // target_brk_press = target_brk_press; // 目标刹车压力, 0~8, 单位0.05MPa
    // byte3~byte4
    uint16_t target_steer_angle = static_cast<uint16_t>(steer_angle + 700.0) * 10;
    if (target_steer_angle > 14000) {
        target_steer_angle = 14000;
    }
    target_steer_angle = target_steer_angle << 1;
    data[3] |= static_cast<uint8_t>((target_steer_angle >> 8) & 0xFF);
    data[4] |= target_steer_angle & 0xFF; // target_steer_angle = target_steer_angle & 0xFF;
    data[4] |= 0x01;                      // steer_moter_enable = 1; // z轴转向电机使能, 1: 启用
    // byte5 跳过 torque模式使用
    // byte6
    data[6] |= 1; // torque_rpm_mode = 1; // 0-扭矩模式，1-速度模式
    uint8_t target_turnlight = 0;
    if (target_steer_angle > 8000) {
        target_turnlight = 1; // 左转
    } else if (target_steer_angle < 6000) {
        target_turnlight = 2; // 右转
    } else {
        target_turnlight = 0; // 关闭
    }
    target_turnlight = 3;
    data[6] |= target_turnlight << 1; // 转向灯
    data[6] |= 2 << 3; // target_spd_mode = 0; // 目标速度模式, 0:低速模式, 1: 中速模式, 2: 高速模式
    // OEC清除标志
    data[6] |= 0 << 5; // oec_clear_flag = 0; // OEC清除标志, 0: 无效, 1: 清除
}

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