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
        this->declare_parameter("intermediate_frame", "base_link");
        this->declare_parameter("target_frame", "map");
        this->declare_parameter("check_period", 1.0);  // 检查周期，单位：秒
        this->declare_parameter("tf_timeout", 1.0);    // 转换超时时间，单位：秒

        // 获取参数
        source_frame_ = this->get_parameter("source_frame").as_string();
        intermediate_frame_ = this->get_parameter("intermediate_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        double check_period = this->get_parameter("check_period").as_double();
        tf_timeout_ = this->get_parameter("tf_timeout").as_double();

        // 创建 TF buffer 和 listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(check_period),
            std::bind(&TransformChecking::checkTransform, this));

        RCLCPP_INFO(this->get_logger(), "Transform checking node initialized");
        RCLCPP_INFO(this->get_logger(), "Checking transform from %s to %s to %s every %.1f seconds",
                    source_frame_.c_str(), intermediate_frame_.c_str(), target_frame_.c_str(), check_period);
        RCLCPP_INFO(this->get_logger(), "Using TF timeout of %.1f seconds", tf_timeout_);
    }

private:
    void checkTransform()
    {
        try {
            // 第一步转换: source_frame (lidar_link) -> intermediate_frame (base_link)
            geometry_msgs::msg::TransformStamped transform_source_to_intermediate;
            
            try {
                transform_source_to_intermediate = tf_buffer_->lookupTransform(
                    intermediate_frame_, source_frame_,
                    tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get latest transform from %s to %s, trying with timeout: %s", 
                           source_frame_.c_str(), intermediate_frame_.c_str(), ex.what());
                transform_source_to_intermediate = tf_buffer_->lookupTransform(
                    intermediate_frame_, source_frame_,
                    rclcpp::Time(0), rclcpp::Duration::from_seconds(tf_timeout_));
            }

            RCLCPP_INFO(this->get_logger(), "Found transform from %s to %s:",
                        source_frame_.c_str(), intermediate_frame_.c_str());
            
            // 打印第一步转换的平移
            RCLCPP_INFO(this->get_logger(), "Translation (step 1): x=%.3f, y=%.3f, z=%.3f",
                        transform_source_to_intermediate.transform.translation.x,
                        transform_source_to_intermediate.transform.translation.y,
                        transform_source_to_intermediate.transform.translation.z);
            
            // 打印第一步转换的旋转（四元数）
            RCLCPP_INFO(this->get_logger(), "Rotation (step 1): x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                        transform_source_to_intermediate.transform.rotation.x,
                        transform_source_to_intermediate.transform.rotation.y,
                        transform_source_to_intermediate.transform.rotation.z,
                        transform_source_to_intermediate.transform.rotation.w);
            
            // 第二步转换: intermediate_frame (base_link) -> target_frame (map)
            geometry_msgs::msg::TransformStamped transform_intermediate_to_target;
            
            try {
                transform_intermediate_to_target = tf_buffer_->lookupTransform(
                    target_frame_, intermediate_frame_,
                    tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get latest transform from %s to %s, trying with timeout: %s", 
                           intermediate_frame_.c_str(), target_frame_.c_str(), ex.what());
                transform_intermediate_to_target = tf_buffer_->lookupTransform(
                    target_frame_, intermediate_frame_,
                    rclcpp::Time(0), rclcpp::Duration::from_seconds(tf_timeout_));
            }

            RCLCPP_INFO(this->get_logger(), "Found transform from %s to %s:",
                        intermediate_frame_.c_str(), target_frame_.c_str());
            
            // 打印第二步转换的平移
            RCLCPP_INFO(this->get_logger(), "Translation (step 2): x=%.3f, y=%.3f, z=%.3f",
                        transform_intermediate_to_target.transform.translation.x,
                        transform_intermediate_to_target.transform.translation.y,
                        transform_intermediate_to_target.transform.translation.z);
            
            // 打印第二步转换的旋转（四元数）
            RCLCPP_INFO(this->get_logger(), "Rotation (step 2): x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                        transform_intermediate_to_target.transform.rotation.x,
                        transform_intermediate_to_target.transform.rotation.y,
                        transform_intermediate_to_target.transform.rotation.z,
                        transform_intermediate_to_target.transform.rotation.w);
            
            // 创建一个测试点
            geometry_msgs::msg::PointStamped point_in;
            point_in.header.frame_id = source_frame_;
            point_in.header.stamp = transform_source_to_intermediate.header.stamp;
            point_in.point.x = 1.0;
            point_in.point.y = 1.0;
            point_in.point.z = 1.0;

            // 第一步：转换点从source_frame到intermediate_frame
            geometry_msgs::msg::PointStamped point_intermediate;
            try {
                tf_buffer_->transform(point_in, point_intermediate, intermediate_frame_);
                
                // 打印中间结果
                RCLCPP_INFO(this->get_logger(), "Test point (1,1,1) transformed to %s: x=%.3f, y=%.3f, z=%.3f",
                            intermediate_frame_.c_str(),
                            point_intermediate.point.x, point_intermediate.point.y, point_intermediate.point.z);
                
                // 第二步：转换点从intermediate_frame到target_frame
                geometry_msgs::msg::PointStamped point_out;
                try {
                    tf_buffer_->transform(point_intermediate, point_out, target_frame_);
                    
                    // 打印最终结果
                    RCLCPP_INFO(this->get_logger(), "Test point transformed to %s: x=%.3f, y=%.3f, z=%.3f",
                                target_frame_.c_str(),
                                point_out.point.x, point_out.point.y, point_out.point.z);
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN(this->get_logger(), "Could not transform point from %s to %s: %s", 
                               intermediate_frame_.c_str(), target_frame_.c_str(), ex.what());
                    
                    // 尝试使用带超时的变换API
                    tf_buffer_->transform(point_intermediate, point_out, target_frame_, 
                                          tf2::durationFromSec(tf_timeout_));
                    RCLCPP_INFO(this->get_logger(), "Test point transformed to %s with timeout: x=%.3f, y=%.3f, z=%.3f",
                                target_frame_.c_str(),
                                point_out.point.x, point_out.point.y, point_out.point.z);
                }
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform point from %s to %s: %s", 
                           source_frame_.c_str(), intermediate_frame_.c_str(), ex.what());
                
                // 尝试使用带超时的变换API进行第一步转换
                try {
                    tf_buffer_->transform(point_in, point_intermediate, intermediate_frame_, 
                                          tf2::durationFromSec(tf_timeout_));
                    RCLCPP_INFO(this->get_logger(), "Test point (1,1,1) transformed to %s with timeout: x=%.3f, y=%.3f, z=%.3f",
                                intermediate_frame_.c_str(),
                                point_intermediate.point.x, point_intermediate.point.y, point_intermediate.point.z);
                    
                    // 继续尝试第二步转换
                    geometry_msgs::msg::PointStamped point_out;
                    tf_buffer_->transform(point_intermediate, point_out, target_frame_, 
                                          tf2::durationFromSec(tf_timeout_));
                    RCLCPP_INFO(this->get_logger(), "Test point transformed to %s with timeout: x=%.3f, y=%.3f, z=%.3f",
                                target_frame_.c_str(),
                                point_out.point.x, point_out.point.y, point_out.point.z);
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to transform point through both steps: %s", ex.what());
                }
            }

            // 直接从source_frame到target_frame的一步转换（用于比较）
            try {
                geometry_msgs::msg::TransformStamped transform_direct = tf_buffer_->lookupTransform(
                    target_frame_, source_frame_, tf2::TimePointZero);
                
                RCLCPP_INFO(this->get_logger(), "Direct transform from %s to %s found for comparison",
                            source_frame_.c_str(), target_frame_.c_str());
                
                geometry_msgs::msg::PointStamped point_direct;
                tf_buffer_->transform(point_in, point_direct, target_frame_);
                
                RCLCPP_INFO(this->get_logger(), "Test point transformed directly to %s: x=%.3f, y=%.3f, z=%.3f",
                            target_frame_.c_str(),
                            point_direct.point.x, point_direct.point.y, point_direct.point.z);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Direct transform not available: %s", ex.what());
            }

        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform chain failed: %s", ex.what());
        }
    }

    std::string source_frame_;
    std::string intermediate_frame_;
    std::string target_frame_;
    double tf_timeout_;
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