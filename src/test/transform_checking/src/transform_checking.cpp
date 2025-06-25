#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TransformChecking : public rclcpp::Node
{
public:
    TransformChecking() : Node("transform_checking")
    {
        // 声明参数
        this->declare_parameter("source_frame", "lidar_link");
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("check_period", 1.0);  // 检查周期，单位：秒

        // 获取参数
        source_frame_ = this->get_parameter("source_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        double check_period = this->get_parameter("check_period").as_double();

        // 创建 TF buffer 和 listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(check_period),
            std::bind(&TransformChecking::checkTransform, this));

        RCLCPP_INFO(this->get_logger(), "Transform checking node initialized");
        RCLCPP_INFO(this->get_logger(), "Checking transform from %s to %s every %.1f seconds",
                    source_frame_.c_str(), target_frame_.c_str(), check_period);
    }

private:
    void checkTransform()
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame_, source_frame_,
                tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(), "Found transform from %s to %s:",
                        source_frame_.c_str(), target_frame_.c_str());
            
            // 打印平移
            RCLCPP_INFO(this->get_logger(), "Translation: x=%.3f, y=%.3f, z=%.3f",
                        transform_stamped.transform.translation.x,
                        transform_stamped.transform.translation.y,
                        transform_stamped.transform.translation.z);
            
            // 打印旋转（四元数）
            RCLCPP_INFO(this->get_logger(), "Rotation (quaternion): x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                        transform_stamped.transform.rotation.x,
                        transform_stamped.transform.rotation.y,
                        transform_stamped.transform.rotation.z,
                        transform_stamped.transform.rotation.w);
            
            // 创建一个测试点
            geometry_msgs::msg::PointStamped point_in;
            point_in.header.frame_id = source_frame_;
            point_in.header.stamp = this->now();
            point_in.point.x = 1.0;
            point_in.point.y = 1.0;
            point_in.point.z = 1.0;

            // 转换点
            geometry_msgs::msg::PointStamped point_out;
            tf_buffer_->transform(point_in, point_out, target_frame_);
            
            // 打印结果
            RCLCPP_INFO(this->get_logger(), "Test point (1,1,1) transformed: x=%.3f, y=%.3f, z=%.3f",
                        point_out.point.x, point_out.point.y, point_out.point.z);

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                        source_frame_.c_str(), target_frame_.c_str(), ex.what());
        }
    }

    std::string source_frame_;
    std::string target_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformChecking>());
    rclcpp::shutdown();
    return 0;
} 