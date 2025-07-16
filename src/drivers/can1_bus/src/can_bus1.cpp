#include "rclcpp/rclcpp.hpp"
#include "can1_bus/can1bus_node.h"
#include <iomanip>
#include <stdint.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <unistd.h>
#include "std_msgs/msg/int32.hpp"
#include <iostream>

namespace can1_bus
{

   Can1busNode::Can1busNode() : Node("can1bus_node")
   {

      InitParams();

      bool ret = InitCansocket(can1_device_name_, can_baudrate_);
      if (!ret)
      {
         RCLCPP_ERROR(this->get_logger(), "CAN socket initialization failed");
         return;
      }

      RCLCPP_INFO(this->get_logger(), "CAN socket initialized successfully");

      // 创建一个线程循环读取CAN数据
      can1_thread_ = std::thread(&Can1busNode::Can1ThreadFunc, this);

      publisher_ = this->create_publisher<std_msgs::msg::Int32>("key_num_topic", 10);

      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Can1busNode::timer_callback, this));
   }

   void Can1busNode::timer_callback()
   {

      auto message = std_msgs::msg::Int32();

      message.data = key_num;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      publisher_->publish(message);
   }

   Can1busNode::~Can1busNode()
   {
      if (can1_thread_.joinable())
      {
         can1_thread_.join();
      }

      if (can_fd > 0)
      {
         close(can_fd);
      }
   }

   void Can1busNode ::InitParams()
   {

      this->declare_parameter<std::string>("can1_device", "can1");
      this->declare_parameter<int>("can1_baud", 250);

      this->get_parameter("can1_device", can1_device_name_);
      this->get_parameter("can1_baud", can_baudrate_);

      RCLCPP_INFO(this->get_logger(), "can1_device_name:%s", can1_device_name_.c_str());
      RCLCPP_INFO(this->get_logger(), "can_baund:%d", can_baudrate_);
   }

   bool Can1busNode::InitCansocket(std::string can_device_name, int baudrate)
   {

      struct sockaddr_can addr;
      struct ifreq ifr;

      if ((can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
      {
         RCLCPP_INFO(this->get_logger(), "Error_Open_Socket:%s", strerror(errno));
         return false;
      }

      strcpy(ifr.ifr_name, can1_device_name_.c_str());

      if ((ioctl(can_fd, SIOCGIFINDEX, &ifr)) < 0)
      {
         RCLCPP_INFO(this->get_logger(), "Error_ioctl %s", strerror(errno));

         close(can_fd);
         return false;
      }

      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      if ((bind(can_fd, (struct sockaddr *)&addr, sizeof(addr))) < 0)
      {
         RCLCPP_INFO(this->get_logger(), "Error_bind%s", strerror(errno));
         close(can_fd);
         return false;
      };

      if ((ioctl(can_fd, SIOCGIFPFLAGS, &ifr)) < 0)
      {
         RCLCPP_INFO(this->get_logger(), "ioctl_2%s", strerror(errno));
      }
      else
      {
         if (ifr.ifr_flags & IFF_UP)
         {
            RCLCPP_INFO(this->get_logger(), "ioctl_2%s", strerror(errno));
         }
         else
         {

            RCLCPP_INFO(this->get_logger(), "ioctl_2%s", strerror(errno));
         }
      }

      RCLCPP_INFO(this->get_logger(), "CAN socket init successfully on %s with baudrate %d",
                  can_device_name.c_str(), baudrate);

      return true;
   }

   void Can1busNode::Can1ThreadFunc()
   {
      fd_set read_fds;

      struct can_frame frame;
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 100000;
      int empty_reads_count = 0;
      int activity;
      int id;

      while (rclcpp::ok())
      {

         FD_ZERO(&read_fds);
         FD_SET(can_fd, &read_fds);

         activity = select(can_fd + 1, &read_fds, NULL, NULL, &tv);
         if (activity < 0)
         {
            RCLCPP_ERROR(this->get_logger(), "Select error: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
         }

         if (activity == 0)
         {
            empty_reads_count++;
            if (empty_reads_count % 100 == 0)
            {
               // RCLCPP_WARN(this->get_logger(), "No CAN data received for ~10 seconds");
            }
            continue;
         }

         if (FD_ISSET(can_fd, &read_fds))
         {
            long unsigned int nbytes = read(can_fd, &frame, sizeof(struct can_frame));
            if (nbytes <= 0)
            {
               if (errno == EAGAIN || errno == EWOULDBLOCK)
               {
                  continue;
               }
               RCLCPP_INFO(this->get_logger(), "Error readinfg canframe:%s", strerror(errno));
               std::this_thread::sleep_for(std::chrono::milliseconds(100));
               continue;
            }

            if (nbytes < sizeof(struct can_frame))
            {
               RCLCPP_INFO(this->get_logger(), "incomplete CAN got%d bytes", nbytes);
            }

            empty_reads_count = 0;

            id = frame.can_id & CAN_EFF_MASK;

            if (id == KEY_CAN_ID)
            {
               PrintCanDataFrame(frame);
               if (frame.data[0] == 0 && frame.data[1] == 0)
               {
                  key_num = 0x00;
               }
               if (frame.data[0] == 0x01)
               {
                  key_num = 0x01;
               }
               if (frame.data[0] == 0x02)
               {
                  key_num = 0x02;
               }
               if (frame.data[0] == 0x04)
               {
                  key_num = 0x03;
               }
               if (frame.data[0] == 0x08)
               {
                  key_num = 0x04;
               }

               if (frame.data[1] == 0x01)
               {
                  key_num = 0x05;
               }
               if (frame.data[1] == 0x02)
               {
                  key_num = 0x06;
               }
            }
         }
      }
   }

   void Can1busNode::PrintCanDataFrame(const struct can_frame &frame)
   {
      std::stringstream ss;
      ss << "CAN frame, ID: 0x" << std::hex << frame.can_id << ", Length: " << std::dec << static_cast<int>(frame.can_dlc)
         << ", Data: ";
      for (int i = 0; i < frame.can_dlc; i++)
      {
         ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << " ";
      }
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
   }

}

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);

   auto node = std::make_shared<can1_bus::Can1busNode>();

   rclcpp::spin(node);

   rclcpp::shutdown();

   return 0;
}