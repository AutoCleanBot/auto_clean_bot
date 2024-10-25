#include "lane_detection/ufld_detector.h"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class UFLDNode : public rclcpp::Node {
  public:
    UFLDNode() : Node("ufld_node") {
        // Declare parameters
        this->declare_parameter("engine_path", "");
        this->declare_parameter("cut_height", 320);
        this->declare_parameter("input_width", 800);
        this->declare_parameter("input_height", 320);
        this->declare_parameter("num_row", 72);
        this->declare_parameter("num_col", 81);

        // Get parameters
        auto enginePath = this->get_parameter("engine_path").as_string();
        ufld_ros::UFLDDetector::Config config;
        config.cutHeight = this->get_parameter("cut_height").as_int();
        config.inputWidth = this->get_parameter("input_width").as_int();
        config.inputHeight = this->get_parameter("input_height").as_int();
        config.numRow = this->get_parameter("num_row").as_int();
        config.numCol = this->get_parameter("num_col").as_int();

        // Initialize row and column anchors
        initializeAnchors(config);

        // Create detector
        detector_ = std::make_unique<ufld_ros::UFLDDetector>(enginePath, config);
        // Create subscriber and publisher
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&UFLDNode::imageCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("lane_detection", 10);

        RCLCPP_INFO(this->get_logger(), "UFLD node initialized");
    }

  private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat result;
            if (detector_->detect(cvPtr->image, result)) {
                auto resultMsg = cv_bridge::CvImage(msg->header, "bgr8", result).toImageMsg();
                publisher_->publish(*resultMsg);
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    void initializeAnchors(ufld_ros::UFLDDetector::Config &config) {
        config.rowAnchor.resize(config.numRow);
        for (int i = 0; i < config.numRow; i++) {
            config.rowAnchor[i] = 0.42 + (1.0 - 0.42) * i / (config.numRow - 1);
        }

        config.colAnchor.resize(config.numCol);
        for (int i = 0; i < config.numCol; i++) {
            config.colAnchor[i] = static_cast<float>(i) / (config.numCol - 1);
        }
    }

    std::unique_ptr<ufld_ros::UFLDDetector> detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UFLDNode>());
    rclcpp::shutdown();
    return 0;
}