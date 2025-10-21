//cpp
#include "RadioLink.h"
#include "rtk.h"
#include "gps_serial.h"
#include <cstdint>
#include <cmath>
#include <iostream> // 可选，用于打印调试

//ros
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cstdio>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>


SBUS_MSG_t sbus_msg;

namespace radio {
RadioNode::RadioNode() : Node("radio_node") {
    // constructor
    InitValues();
    InitParams();
    bool ret = DeviceInit();
    //判断串口是否初始化成功
    if (ret) {
        running_ = true;
        //启动一个后台线程执行InfoReadLoop函数
        read_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&RadioNode::InfoReadLoop, this)));
        read_thread_->detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize device");
    }

    // publishers and subscribers //create_wall_timer和create_publisher 是node类的成员函数
    int time_interval = static_cast<int>(1000.0 / radio_publish_rate_); //20ms
    radio_timer_ = 
        this->create_wall_timer(std::chrono::milliseconds(time_interval), std::bind(&RadioNode::RadioTimerCallback, this));

    pub_radio_info_ = this->create_publisher<bot_msg::msg::RadioLink>(radio_topic_name_, 10);
}

RadioNode::~RadioNode() {
    // 停止运行标志
    running_ = false;
    
    // 关闭info_str日志文件
    if (info_str_file_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Closing info_str log file. Total records saved: %d", info_str_save_count_);
        info_str_file_.close();
    }
    
    // 关闭串口
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

void RadioNode::RadioTimerCallback() {
    if (running_ && base_point_set_) {
        pub_radio_info_->publish(radio_msg_);
    }
}

//从串口读取数据的线程函数
void RadioNode::InfoReadLoop() {
    std::string data;
    std::string data_all;
    int i = 1;
    // std::string gstart = "$"; // 开头
    // std::string gend = "\r\n";
    while (running_) {
        char buf[512] = {0};
        int ret = read(sockfd_, buf, sizeof(buf)); //返回读取的字节数
        
        // std::cout << "Data length: " << ret << std::endl;
        // std::cout << "Data length1: " << buf << std::endl;
        // if (ret < 0) {
        //     error_count_++;
        //     // 限制错误日志频率：每50次错误输出一次，或者每5秒输出一次
        //     auto now = std::chrono::steady_clock::now();
        //     auto time_since_last_log = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_).count();

        //     if ((error_count_ % 50 == 0) || (time_since_last_log >= 5)) {
        //         RCLCPP_WARN(this->get_logger(), "Read error (count: %d): %s", error_count_, strerror(errno));
        //         last_log_time_ = now;
        //     }
        // }    
        data.clear();
        data += buf;
        // std::cout << "Data length: " << data.size() << std::endl;
        if (!data.empty()) {  
            data_all+=data;
            i++;

            // 打印当前 data 的十六进制内容
            std::stringstream ss;
            for (char c : data) {
                ss << std::hex << std::uppercase << std::setw(2)
                   << std::setfill('0')
                   << static_cast<int>(static_cast<unsigned char>(c))
                   << " ";
            }
            std::cout << "[Current Data Hex] " << ss.str() << std::endl;

            // 每 3 次打印和清空累计数据
            if (i % 3 == 0) {
                // 打印累计 data_all 的十六进制内容
                std::stringstream ss_all;
                for (char c : data_all) {
                    ss_all << std::hex << std::uppercase << std::setw(2)
                        << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(c))
                        << " ";
                }
                std::cout << "[Accumulated Hex] " << ss_all.str() << std::endl;
                std::cout << "---- Clearing accumulated data ----" << std::endl;
                data_all.clear();
            }

        }

        // auto start_pos = data.find(gstart);
        // auto end_pos = data.find(gend);
        // if (start_pos != std::string::npos && end_pos != std::string::npos) {
        //     std::string info_str = data.substr(start_pos, end_pos - start_pos + gend.length());
            
        //     // Save info_str to file if enabled
        //     if (enable_info_str_save_) {
        //         SaveInfoStrToFile(info_str);
        //     }
            
        //     // ParseRadioInfo(info_str);
        //     //核心函数
        //     Radio_AnalyzeLoop();

        //     data.erase(0, end_pos + gend.length());
        // }
    }
}

/**
 * @brief configure the node with parameters
 * 节点参数配置
 */
void RadioNode::InitParams() {
// declare parameters here
    this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 100000);
    this->declare_parameter<int>("timeout_ms", 20);
    this->declare_parameter<bool>("enable_debug_log", false);
    this->declare_parameter<int>("log_interval", 10); // 每10次解析输出一次日志

    this->declare_parameter<std::string>("radio_frame_id", "radio_frame");
    this->declare_parameter<std::string>("radio_topic_name", "radio_fix");
    this->declare_parameter<double>("radio_publish_rate", 10.0);
    
    // Parameters for saving info_str to file
    this->declare_parameter<bool>("enable_info_str_save", false);
    this->declare_parameter<std::string>("info_str_save_dir", "/tmp/radio_logs");

    // set the parameters
    this->get_parameter("device_name", this->device_name_);
    this->get_parameter("baud_rate", this->baud_rate_);
    this->get_parameter("timeout_ms", this->timeout_ms_);
    this->get_parameter("radio_frame_id", this->radio_frame_id_);
    this->get_parameter("radio_topic_name", this->radio_topic_name_);
    this->get_parameter("radio_publish_rate", this->radio_publish_rate_);

    this->get_parameter("enable_debug_log", this->enable_debug_log_);
    this->get_parameter("log_interval", this->log_interval_);
    
    this->get_parameter("enable_info_str_save", this->enable_info_str_save_);
    this->get_parameter("info_str_save_dir", this->info_str_save_dir_);

    // 初始化日志频率控制变量
    parse_count_ = 0;
    error_count_ = 0;
    last_log_time_ = std::chrono::steady_clock::now();

    // print the parameters
    RCLCPP_INFO(this->get_logger(), "Device name: %s", this->device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", this->baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Timeout (ms): %d", this->timeout_ms_);
    RCLCPP_INFO(this->get_logger(), "radio frame id: %s", this->radio_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "radio topic name: %s", this->radio_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "radio publish rate: %lf", this->radio_publish_rate_);

    RCLCPP_INFO(this->get_logger(), "Enable debug log: %d", this->enable_debug_log_);
    RCLCPP_INFO(this->get_logger(), "Log interval: %d (every %d parses)", this->log_interval_, this->log_interval_);

    RCLCPP_INFO(this->get_logger(), "Enable info_str save: %d", this->enable_info_str_save_);
    RCLCPP_INFO(this->get_logger(), "Info_str save directory: %s", this->info_str_save_dir_.c_str());
    
    // Initialize info_str saving functionality
    if (enable_info_str_save_) {
        InitInfoStrSaving();
    }
    
    return;
}

/**
 * @brief initialize the device
 *
 * @return true
 * @return false
 */
bool RadioNode::DeviceInit() {
    // initialize device here

    // 串口设置
    // serial_port_.setPort(device_name_);
    sockfd_ = Serial::TTYOpen(device_name_.c_str());
    if (sockfd_ < 0) {
        sockfd_ = -1;
        RCLCPP_ERROR(this->get_logger(), "tty_open failed.");
        return false;
    }
    if (Serial::TTYSetOpt(sockfd_, baud_rate_, 8, 1, 'n') != 0) {
        sockfd_ = -1;
        RCLCPP_ERROR(this->get_logger(), "tty_setopt error.");
        return false;
    }

    return true;
}

//关键变量初始化
void RadioNode::InitValues() {
    // initialize values here
    base_point_set_ = true;
    running_ = false;
    read_thread_ = nullptr;
    info_str_save_count_ = 0;
}

//保存初始化参数的log
void RadioNode::InitInfoStrSaving() {
    // Create directory if it doesn't exist
    try {
        std::filesystem::create_directories(info_str_save_dir_);
        RCLCPP_INFO(this->get_logger(), "Info_str save directory created/verified: %s", info_str_save_dir_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create info_str save directory: %s", e.what());
        enable_info_str_save_ = false;
    }
}

void RadioNode::SaveInfoStrToFile(const std::string &info_str) {
    // Generate new filename if needed (first time or file doesn't exist)
    if (current_log_filename_.empty() || !info_str_file_.is_open()) {
        current_log_filename_ = GenerateTimestampFilename();
        std::string full_path = info_str_save_dir_ + "/" + current_log_filename_;
        
        // Close previous file if open
        if (info_str_file_.is_open()) {
            info_str_file_.close();
        }
        
        // Open new file
        info_str_file_.open(full_path, std::ios::app);
        if (!info_str_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open info_str log file: %s", full_path.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Started logging info_str to: %s", full_path.c_str());
    }
    
    // Write raw info_str to file (不添加时间戳，直接保存原始数据)
    info_str_file_ << info_str << std::endl;
    info_str_file_.flush();
    
    info_str_save_count_++; // 仅用于统计，不用于过滤
}

std::string RadioNode::GenerateTimestampFilename() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << "Radio_info_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".log";
    return ss.str();
}
}


void Radio_init(void)
{
    sbus_msg.data[23] = 0xFF;
    int8_t i;
    for (i = 0; i < 24; i++)
    {
        sbus_msg.data[i] = 900;
    }
}
void Radio_enqueue(uint8_t data) //遥控器接收
{
    sbus_msg.data[sbus_msg.flag++] = data;
    if (sbus_msg.flag >= 25)
    {
        sbus_msg.flag = 0;
        sbus_msg.ready = 1;
    }
}
void Radio_AnalyzeLoop(void) //不断调用 接收遥控器25次调用一次
{
    if (sbus_msg.ready == 0)
    {
        return;
    }
    if (sbus_msg.data[0] != 0x0F)
    {
        sbus_msg.flag = 0;
        return;
    }
    sbus_msg.CH[0] = -1.25 + 0.00125 * ((sbus_msg.data[1] | sbus_msg.data[2] << 8) & 0x07FF);
    sbus_msg.CH[1] = -1.25 + 0.00125 * ((sbus_msg.data[2] >> 3 | sbus_msg.data[3] << 5) & 0x07FF);
    sbus_msg.CH[2] = -(-1.25 + 0.00125 * ((sbus_msg.data[3] >> 6 | sbus_msg.data[4] << 2 | sbus_msg.data[5] << 10) & 0x07FF));
    sbus_msg.CH[3] = -1.25 + 0.00125 * ((sbus_msg.data[5] >> 1 | sbus_msg.data[6] << 7) & 0x07FF);
    sbus_msg.CH[4] = -1.25 + 0.00125 * ((sbus_msg.data[6] >> 4 | sbus_msg.data[7] << 4) & 0x07FF);
    sbus_msg.CH[5] = -1.25 + 0.00125 * ((sbus_msg.data[7] >> 7 | sbus_msg.data[8] << 1 | sbus_msg.data[9] << 9) & 0x07FF);
    sbus_msg.CH[6] = -1.25 + 0.00125 * ((sbus_msg.data[9] >> 2 | sbus_msg.data[10] << 6) & 0x07FF);
    sbus_msg.CH[7] = -1.25 + 0.00125 * ((sbus_msg.data[10] >> 5 | sbus_msg.data[11] << 3) & 0x07FF);
    sbus_msg.CH[8] = -1.25 + 0.00125 * ((sbus_msg.data[12] | sbus_msg.data[13] << 8) & 0x07FF);
    sbus_msg.CH[9] = -1.25 + 0.00125 * ((sbus_msg.data[13] >> 3 | sbus_msg.data[14] << 5) & 0x07FF);
    sbus_msg.CH[10] = -1.25 + 0.00125 * ((sbus_msg.data[14] >> 6 | sbus_msg.data[15] << 2 | sbus_msg.data[16] << 10) & 0x07FF);
    sbus_msg.CH[11] = -1.25 + 0.00125 * ((sbus_msg.data[16] >> 1 | sbus_msg.data[17] << 7) & 0x07FF);
    sbus_msg.CH[12] = -1.25 + 0.00125 * ((sbus_msg.data[17] >> 4 | sbus_msg.data[18] << 4) & 0x07FF);
    sbus_msg.CH[13] = -1.25 + 0.00125 * ((sbus_msg.data[18] >> 7 | sbus_msg.data[19] << 1 | sbus_msg.data[20] << 9) & 0x07FF);
    sbus_msg.CH[14] = -1.25 + 0.00125 * ((sbus_msg.data[20] >> 2 | sbus_msg.data[21] << 6) & 0x07FF);
    sbus_msg.CH[15] = -1.25 + 0.00125 * ((sbus_msg.data[21] >> 5 | sbus_msg.data[22] << 3) & 0x07FF);
    sbus_msg.CH[0] = FLOAT_LIMIT(sbus_msg.CH[0]);
    sbus_msg.CH[1] = FLOAT_LIMIT(sbus_msg.CH[1]);
    sbus_msg.CH[2] = FLOAT_LIMIT(sbus_msg.CH[2]);
    sbus_msg.CH[3] = FLOAT_LIMIT(sbus_msg.CH[3]);
    sbus_msg.CH[4] = FLOAT_LIMIT(sbus_msg.CH[4]);
    sbus_msg.CH[5] = FLOAT_LIMIT(sbus_msg.CH[5]);
    sbus_msg.CH[6] = FLOAT_LIMIT(sbus_msg.CH[6]);
    sbus_msg.CH[7] = FLOAT_LIMIT(sbus_msg.CH[7]);
    sbus_msg.CH[8] = FLOAT_LIMIT(sbus_msg.CH[8]);
    sbus_msg.CH[9] = FLOAT_LIMIT(sbus_msg.CH[9]);
    sbus_msg.CH[10] = FLOAT_LIMIT(sbus_msg.CH[10]);
    sbus_msg.CH[11] = FLOAT_LIMIT(sbus_msg.CH[11]);
    sbus_msg.CH[12] = FLOAT_LIMIT(sbus_msg.CH[12]);
    sbus_msg.CH[13] = FLOAT_LIMIT(sbus_msg.CH[13]);
    sbus_msg.CH[14] = FLOAT_LIMIT(sbus_msg.CH[14]);
    sbus_msg.CH[15] = FLOAT_LIMIT(sbus_msg.CH[15]);

    sbus_msg.flags.R = sbus_msg.data[23];
    sbus_msg.ready = 0;
}

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "radiolink test!");

    auto node = std::make_shared<radio::RadioNode>();
    RCLCPP_INFO(node->get_logger(), "Radio node started");

    // 循环等待消息
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
    
}