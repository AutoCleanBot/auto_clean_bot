#include "canbus/canbus_node.h"

#include <iomanip>
#include <chrono>


namespace canbus {
CanbusNode::CanbusNode() : Node("canbus_node") {
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
    running_ = false;  // 设置标志位通知线程退出
    if (can_thread_.joinable()) {
        can_thread_.join();  // 等待线程结束
    }
    if (can_fd_ > 0) {
        close(can_fd_);
    }
}

void CanbusNode::TimerCallback() {
    auto msg = std::make_shared<bot_msg::msg::ChassisInfo>();
    FillChassisInfo(msg);
    pub_chassis_info_->publish(*msg);
}

/**
 * 填充底盘信息
 */
void CanbusNode::FillChassisInfo(bot_msg::msg::ChassisInfo::SharedPtr msg) {
    msg->steer_angle = (static_cast<float>(chassis_info_local_.cur_steer_angle) - 3000.0f) / 10.0f;
    msg->brk_press = static_cast<float>(chassis_info_local_.cur_brk_press) * 0.05f / 8.0f; // 压力百分比
    msg->cur_speed = static_cast<float>(chassis_info_local_.current_speed) / 10.0f;
    msg->soc = static_cast<float>(chassis_info_local_.soc) / 100.0f;
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
        RCLCPP_ERROR(this->get_logger(), "Error while opening socket");
        return false;
    }

    // 指定CAN设备名称
    strcpy(ifr.ifr_name, can_device_name.c_str());
    ioctl(can_fd_, SIOCGIFINDEX, &ifr);

    // 绑定套接字
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
        return false;
    }
    // // 启动CAN设备
    // if (ioctl(can_fd_, SIOCGIFFLAGS, &ifr) < 0) {
    //     RCLCPP_ERROR(this->get_logger(), "Error getting flags");
    //     return false;
    // }
    // ifr.ifr_flags |= IFF_UP;
    // if (ioctl(can_fd_, SIOCSIFFLAGS, &ifr) < 0) {
    //     RCLCPP_ERROR(this->get_logger(), "Error setting flags");
    //     return false;
    // }

    RCLCPP_INFO(this->get_logger(), "CAN socket initialized successfully on %s with baudrate %d",
                can_device_name.c_str(), can_baudrate);
    return true;
}

/**
 * 单独的线程循环读取CAN数据
 */

void CanbusNode::CanThreadFunc() {
    while (running_ && rclcpp::ok()) {
        // 读取CAN数据
        struct can_frame frame;
        int nbytes = read(can_fd_, &frame, sizeof(frame));
        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error reading CAN frame");
            continue;
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
            chassis_info_local_.soc = vcu_info_2.soc;
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
            chassis_info_local_.current_speed = vcu_info_spd.current_speed;
            chassis_info_local_.current_direction = vcu_info_spd.current_direction;
            chassis_info_local_.current_gear = vcu_info_spd.current_gear;
        } else if (frame.can_id == SEB_INFO) {
            // 解析SEB_INFO
            CanSEBInfo seb_info;
            memcpy(&seb_info, frame.data, sizeof(seb_info));
            chassis_info_local_.cur_brk_press = seb_info.cur_brk_press;
        } else if (frame.can_id == SES_INFO) {
            // 解析SES_INFO
            CanSESInfo ses_info;
            memcpy(&ses_info, frame.data, sizeof(ses_info));
            chassis_info_local_.cur_steer_angle = ses_info.cur_steer_angle;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown CAN frame id: %d", frame.can_id);
        }
    }
}

/**
 * 控制指令回调函数, 目前的策略是直接转发来自于上层控制器的控制指令
 */
void CanbusNode::ControlCmdCallback(const bot_msg::msg::ControlCmd::SharedPtr msg) {
    // 解析控制指令
    double steer_angle = msg->steer_angle;
    double brk = msg->brk;
    // double thr = msg->thr;
    uint8_t gear = msg->gear;
    double spd = msg->speed;
    // 解析控制指令
    CanCtrlCmd ctrl_cmd;
    FillCanCtrlCmd(ctrl_cmd, steer_angle, brk, gear, spd);

    // 发送控制指令
    can_frame frame;
    frame.can_id = CONTROL_CMD;
    frame.can_dlc = sizeof(ctrl_cmd);
    memcpy(frame.data, &ctrl_cmd, sizeof(ctrl_cmd));
    int ret = write(can_fd_, &frame, sizeof(frame));
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame");
    }
}
/**
 * 填充控制CAN结构体
 */
void CanbusNode::FillCanCtrlCmd(CanCtrlCmd &ctrl_cmd, double steer_angle, double brk,uint8_t gear,
                                double spd) {
    static uint8_t life_signal = 0;
    // byte0
    ctrl_cmd.motor_enable = 1;   // 1: 启用
    ctrl_cmd.target_gear = gear; // 目标档位, 0: N挡, 1: D档, 2: R档
    ctrl_cmd.target_mode = 1;    // 1: 线控模式
    ctrl_cmd.life_signal = life_signal++;
    if (life_signal > 15) {
        life_signal = 0;
    }
    // byte1
    uint8_t target_speed = (spd * 10 > 127) ? 127 : spd * 10;
    ctrl_cmd.target_speed = target_speed; // 目标速度, 0~127, 单位0.1m/s
    ctrl_cmd.target_spd_val = 1;          // 目标速度值有效位, 1: 有效

    // byte2
    uint8_t target_brk_press = 0;
    if (spd <= 0.1) {
        target_brk_press = 1;
    } else {
        // 下发为百分比, 转换为0~8,单位0.05MPa
        target_brk_press = static_cast<uint8_t>(brk * 8 * 20);
    }
    ctrl_cmd.target_brk_press = target_brk_press; // 目标刹车压力, 0~8, 单位0.05MPa
    // byte3~4
    uint16_t target_steer_angle = static_cast<uint16_t>(steer_angle * 10.0 + 700.0);
    if (target_steer_angle > 1400) {
        target_steer_angle = 1400;
    } 
    // 左正右负
    ctrl_cmd.target_steer_angle = target_steer_angle; // 目标转向角度, -700~700, 单位0.1度, 偏移量为-700度
    ctrl_cmd.steer_moter_enable = 1;                  // z轴转向电机使能, 1: 启用

    // byte5
    // 目前使用速度模式
    ctrl_cmd.target_torque = 0;
    // byte6
    ctrl_cmd.torque_rpm_mode = 1; // 扭矩转速模式, 1: 速度模式
    // 转向灯
    if (target_steer_angle > 800) {
        ctrl_cmd.target_turnlight = 1; // 左转
    } else if (target_steer_angle < 600) {
        ctrl_cmd.target_turnlight = 2; // 右转
    } else {
        ctrl_cmd.target_turnlight = 0; // 关闭
    }
    // 速度模式
    ctrl_cmd.target_spd_mode = 0; // 目标速度模式, 0:低速模式, 1: 中速模式, 2: 高速模式
    // OEC清除标志
    ctrl_cmd.oec_clear_flag = 0; // OEC清除标志, 0: 无效, 1: 清除
}
} // namespace canbus

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<canbus::CanbusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}