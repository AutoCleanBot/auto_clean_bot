#include "canbus/canbus_node.h"

namespace canbus {

CanbusNode::CanbusNode() : Node("canbus_node") {
    InitParams();
    bool ret = InitCanSocket(can_device_name_, can_baud_);
    if (!ret) {
        RCLCPP_ERROR(this->get_logger(), "CAN socket initialization failed");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "CAN socket initialized successfully");

    // 初始化订阅者和发布者
    control_cmd_sub_ = this->create_subscription<ControlCmd>(
        control_cmd_topic_, 10, std::bind(&CanbusNode::ControlCmdCallback, this, std::placeholders::_1));
    chassis_info_pub_ = this->create_publisher<ChassisInfo>(chassis_info_topic_, 10);

    // 创建一个线程循环读取CAN数据
    auto can_thread_ = std::thread(&CanbusNode::CanThreadFunc, this);
}
void CanbusNode::InitParams() {
    this->declare_parameter<std::string>("can_device", "can0");
    this->declare_parameter<int>("can_baud", 500);
    this->declare_parameter<int>("control_cmd_topic", "/control_cmd");
    this->declare_parameter<int>("chassis_info_topic", "/chassis_info_topic");

    this->get_parameter("can_device", can_device_name_);
    this->get_parameter("can_baud", can_baud_);
    this->get_parameter("control_cmd_topic", control_cmd_topic_);
    this->get_parameter("chassis_info_topic", chassis_info_topic_);

    RCLCPP_INFO(this->get_logger(), "can_device_name: %s", can_device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "can_baud: %d", can_baud_);
    RCLCPP_INFO(this->get_logger(), "control_cmd_topic: %s", control_cmd_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "chassis_info_topic: %s", chassis_info_topic_.c_str());
}

bool InitCanSocket(std::string can_device_name, int can_baudrate) {
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

    // 设置CAN波特率
    struct can_bittiming bit;
    bit.bitrate = can_baudrate;
    if (ioctl(can_fd_, SIOCSCANBAUDRATE, &bit) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting baudrate");
        return false;
    }

    // 启动CAN设备
    if (ioctl(can_fd_, SIOCGIFFLAGS, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting flags");
        return false;
    }
    ifr.ifr_flags |= IFF_UP;
    if (ioctl(can_fd_, SIOCSIFFLAGS, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting flags");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "CAN socket initialized successfully on %s with baudrate %d",
                can_device_name.c_str(), can_baudrate);
    return true;
}

void CanbusNode::CanThreadFunc() {
    while (rclcpp::ok()) {
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
        } else if (frame.can_id == VCU_INFO_2) {
        } else if (frame.can_id == VCU_INFO_DIAG) {
        } else if (frame.can_id == VCU_INFO_SPD) {
        }else if(frame.can_id == SEB_INFO){
        }else if(frame.can_id == SES_INFO){
        }else{
            RCLCPP_WARN(this->get_logger(), "Unknown CAN frame id: %d", frame.can_id);
        }
    }
}

void CanbusNode::ControlCmdCallback(const ControlCmd::SharedPtr msg) {
    // 解析控制指令
}

} // namespace canbus