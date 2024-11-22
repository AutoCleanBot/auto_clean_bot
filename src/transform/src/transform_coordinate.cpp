#include "rclcpp/rclcpp.hpp"  
#include "std_msgs/msg/string.hpp" 
#include "corner_radar_transform.hpp"
#include "lidar_transform.hpp"
#include "camera_transform.hpp"
#include <yaml.h>

using namespace std::chrono_literals;  
  
class TransformCoordinateNode : public rclcpp::Node  
{  
public:  
    TransformCoordinateNode()  
        : Node("node_transform_coordinate"),count(0) 
    {  
        //参数初始化
        init_parameters();

        //left_top_corner_radar
        left_top_corner_radar_transform = std::make_unique<CornerRadarTransform>(left_top_corner_radar_x,left_top_corner_radar_y,
                                                                             left_top_corner_radar_z,left_top_corner_radar_theta,
                                                                             CornerType::LEFT_FRONT);
        //right_top_corner_radar
        right_top_corner_radar_transform = std::make_unique<CornerRadarTransform>(right_top_corner_radar_x,right_top_corner_radar_y,
                                                                              right_top_corner_radar_z,right_top_corner_radar_theta,
                                                                              CornerType::RIGHT_FRONT);
        //left_back_corner_radar
        left_back_corner_radar_transform = std::make_unique<CornerRadarTransform>(left_back_corner_radar_x,left_back_corner_radar_y,
                                                                             left_back_corner_radar_z,left_back_corner_radar_theta,
                                                                             CornerType::LEFT_BACK);
        //right_back_corner_radar
        right_back_corner_radar_transform = std::make_unique<CornerRadarTransform>(right_back_corner_radar_x,right_back_corner_radar_y,
                                                                              right_back_corner_radar_z,right_back_corner_radar_theta,
                                                                              CornerType::RIGHT_BACK);
        //top_fornt_corner_radar
        top_fornt_corner_radar_transform = std::make_unique<CornerRadarTransform>(top_fornt_corner_radar_x,top_fornt_corner_radar_y,
                                                                             top_fornt_corner_radar_z,top_fornt_corner_radar_theta,
                                                                             CornerType::TOP_FRONT);
        //lidar
        lidar_transform = std::make_unique<LidarTransform>(lidar_yaw,lidar_pitch,lidar_roll,lidar_x,lidar_y,lidar_z);
        RCLCPP_INFO(this->get_logger(), "lidar_transform init success");

        //camera
        camera_transform = std::make_unique<CameraTransform>();
        camera_transform->set_camera_intrinsic_params(camera_fx,camera_fy,camera_cx,camera_cy);
        camera_transform->set_camera_extrinsic_params(camera_yaw,camera_pitch,camera_roll,camera_x,camera_y,camera_z);
        RCLCPP_INFO(this->get_logger(), "camera_transform init success"); 



        // // 创建订阅者  订阅控制指令
        // subscription = this->create_subscription<ControlCmd>(  
        //     "/controlCmd", 10,  
        //     std::bind(&CanNode::controlCmd_callback, this, std::placeholders::_1));  

        // // 创建发布者  发布底盘信息
        // publisher = create_publisher<ChassisInfo>("/chassisInfo", 10);
    }

    ~TransformCoordinateNode()  
    {  

    }  
  
private:
    void init_parameters()
    {
        // 读取参数
        declare_parameter("left_top_corner_radar_theta",left_top_corner_radar_theta);
        declare_parameter("left_top_corner_radar_x",left_top_corner_radar_x);
        declare_parameter("left_top_corner_radar_y",left_top_corner_radar_y);
        declare_parameter("left_top_corner_radar_z",left_top_corner_radar_z);
        declare_parameter("right_top_corner_radar_theta",right_top_corner_radar_theta);
        declare_parameter("right_top_corner_radar_x",right_top_corner_radar_x);
        declare_parameter("right_top_corner_radar_y",right_top_corner_radar_y);
        declare_parameter("right_top_corner_radar_z",right_top_corner_radar_z);
        declare_parameter("left_back_corner_radar_theta",left_back_corner_radar_theta);
        declare_parameter("left_back_corner_radar_x",left_back_corner_radar_x);
        declare_parameter("left_back_corner_radar_y",left_back_corner_radar_y);
        declare_parameter("left_back_corner_radar_z",left_back_corner_radar_z);
        declare_parameter("right_back_corner_radar_theta",right_back_corner_radar_theta);
        declare_parameter("right_back_corner_radar_x",right_back_corner_radar_x);
        declare_parameter("right_back_corner_radar_y",right_back_corner_radar_y);
        declare_parameter("right_back_corner_radar_z",right_back_corner_radar_z);
        declare_parameter("top_fornt_corner_radar_theta",top_fornt_corner_radar_theta);
        declare_parameter("top_fornt_corner_radar_x",top_fornt_corner_radar_x);
        declare_parameter("top_fornt_corner_radar_y",top_fornt_corner_radar_y);
        declare_parameter("top_fornt_corner_radar_z",top_fornt_corner_radar_z);
        declare_parameter("lidar_yaw",lidar_yaw);
        declare_parameter("lidar_pitch",lidar_pitch);
        declare_parameter("lidar_roll",lidar_roll);
        declare_parameter("lidar_x",lidar_x);
        declare_parameter("lidar_y",lidar_y);
        declare_parameter("lidar_z",lidar_z);
        declare_parameter("camera_fx",camera_fx);
        declare_parameter("camera_fy",camera_fy);
        declare_parameter("camera_cx",camera_cx);
        declare_parameter("camera_cy",camera_cy);
        declare_parameter("camera_yaw",camera_yaw);
        declare_parameter("camera_pitch",camera_pitch);
        declare_parameter("camera_roll",camera_roll);        
        declare_parameter("camera_x",camera_x);
        declare_parameter("camera_y",camera_y);
        declare_parameter("camera_z",camera_z);

        get_parameter("left_top_corner_radar_theta",left_top_corner_radar_theta);
        get_parameter("left_top_corner_radar_x",left_top_corner_radar_x);
        get_parameter("left_top_corner_radar_y",left_top_corner_radar_y);
        get_parameter("left_top_corner_radar_z",left_top_corner_radar_z);
        get_parameter("right_top_corner_radar_theta",right_top_corner_radar_theta);
        get_parameter("right_top_corner_radar_x",right_top_corner_radar_x);
        get_parameter("right_top_corner_radar_y",right_top_corner_radar_y);
        get_parameter("right_top_corner_radar_z",right_top_corner_radar_z);
        get_parameter("left_back_corner_radar_theta",left_back_corner_radar_theta);
        get_parameter("left_back_corner_radar_x",left_back_corner_radar_x);
        get_parameter("left_back_corner_radar_y",left_back_corner_radar_y);
        get_parameter("left_back_corner_radar_z",left_back_corner_radar_z);
        get_parameter("right_back_corner_radar_theta",right_back_corner_radar_theta);
        get_parameter("right_back_corner_radar_x",right_back_corner_radar_x);
        get_parameter("right_back_corner_radar_y",right_back_corner_radar_y);
        get_parameter("right_back_corner_radar_z",right_back_corner_radar_z);
        get_parameter("top_fornt_corner_radar_theta",top_fornt_corner_radar_theta);
        get_parameter("top_fornt_corner_radar_x",top_fornt_corner_radar_x);
        get_parameter("top_fornt_corner_radar_y",top_fornt_corner_radar_y);
        get_parameter("top_fornt_corner_radar_z",top_fornt_corner_radar_z);
        get_parameter("lidar_yaw",lidar_yaw);
        get_parameter("lidar_pitch",lidar_pitch);
        get_parameter("lidar_roll",lidar_roll);
        get_parameter("lidar_x",lidar_x);
        get_parameter("lidar_y",lidar_y);        
        get_parameter("lidar_z",lidar_z);
        get_parameter("camera_fx",camera_fx);
        get_parameter("camera_fy",camera_fy);
        get_parameter("camera_cx",camera_cx);
        get_parameter("camera_cy",camera_cy);
        get_parameter("camera_yaw",camera_yaw);
        get_parameter("camera_pitch",camera_pitch);
        get_parameter("camera_roll",camera_roll);
        get_parameter("camera_x",camera_x);
        get_parameter("camera_y",camera_y);
        get_parameter("camera_z",camera_z);

        // 打印参数  
        RCLCPP_INFO(this->get_logger(), "Left Top Corner Radar: (%f,%f,%f,%f)", left_top_corner_radar_x, left_top_corner_radar_y, left_top_corner_radar_z, left_top_corner_radar_theta);  
        RCLCPP_INFO(this->get_logger(), "Right Top Corner Radar: (%f,%f,%f,%f)", right_top_corner_radar_x, right_top_corner_radar_y, right_top_corner_radar_z, right_top_corner_radar_theta);  
        RCLCPP_INFO(this->get_logger(), "Left Back Corner Radar: (%f,%f,%f,%f)", left_back_corner_radar_x, left_back_corner_radar_y, left_back_corner_radar_z, left_back_corner_radar_theta);  
        RCLCPP_INFO(this->get_logger(), "Right Back Corner Radar: (%f,%f,%f,%f)", right_back_corner_radar_x, right_back_corner_radar_y, right_back_corner_radar_z, right_back_corner_radar_theta);  
        RCLCPP_INFO(this->get_logger(), "Top Front Corner Radar: (%f,%f,%f,%f)", top_fornt_corner_radar_x, top_fornt_corner_radar_y, top_fornt_corner_radar_z, top_fornt_corner_radar_theta);        
        RCLCPP_INFO(this->get_logger(), "Lidar: (%f,%f,%f,%f,%f,%f)", lidar_x, lidar_y, lidar_z, lidar_roll, lidar_pitch, lidar_yaw);   
        RCLCPP_INFO(this->get_logger(), "Camera: (%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)", camera_fx, camera_fy, camera_cx, camera_cy, camera_yaw, camera_pitch, camera_roll, camera_x, camera_y, camera_z);   

    }
    // void timer_callback()
    // {
    //     // VehicleData vehicleData =  canCtrl->getChassisInfo();
    //     // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", vehicleData.readytatus); 

    //     auto chassisInfo = ChassisInfo();
    //     // message.data = "Hello, world! ";
    //     RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count++);
    //     //底盘信息
    //     publisher->publish(chassisInfo);
    // }  
    // void controlCmd_callback(const ControlCmd::SharedPtr msg) const  
    // {  
    //     //can_device->setControlCmdInfo(msg);  
    //     RCLCPP_INFO(this->get_logger(), "Speed:%d'", msg->speed);  
    // }  
    // 类的实例  
    std::unique_ptr<CornerRadarTransform> left_top_corner_radar_transform; 
    std::unique_ptr<CornerRadarTransform> right_top_corner_radar_transform; 
    std::unique_ptr<CornerRadarTransform> left_back_corner_radar_transform; 
    std::unique_ptr<CornerRadarTransform> right_back_corner_radar_transform;
    std::unique_ptr<CornerRadarTransform> top_fornt_corner_radar_transform;
    std::unique_ptr<LidarTransform> lidar_transform;
    std::unique_ptr<CameraTransform> camera_transform;
    int count;  
    //parameters 
    //left_top_corner_radar 
    float left_top_corner_radar_theta;
    float left_top_corner_radar_x;
    float left_top_corner_radar_y;
    float left_top_corner_radar_z;
    //right_top_corner_radar 
    float right_top_corner_radar_theta;
    float right_top_corner_radar_x;
    float right_top_corner_radar_y;
    float right_top_corner_radar_z;
    //left_back_corner_radar 
    float left_back_corner_radar_theta;
    float left_back_corner_radar_x;
    float left_back_corner_radar_y;
    float left_back_corner_radar_z;
    //right_back_corner_radar 
    float right_back_corner_radar_theta;
    float right_back_corner_radar_x;
    float right_back_corner_radar_y;
    float right_back_corner_radar_z;
    //top_fornt_corner_radar 
    float top_fornt_corner_radar_theta;
    float top_fornt_corner_radar_x;
    float top_fornt_corner_radar_y;
    float top_fornt_corner_radar_z;
    //lidar
    float lidar_yaw;
    float lidar_pitch;
    float lidar_roll;
    float lidar_x;
    float lidar_y;
    float lidar_z;
    //camara
    float camera_fx;
    float camera_fy;
    float camera_cx;
    float camera_cy;
    float camera_yaw;
    float camera_pitch;
    float camera_roll;
    float camera_x;
    float camera_y;
    float camera_z;

    std::vector<CornerRadar> corner_radars_;
};  
  
int main(int argc, char *argv[])  
{  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<TransformCoordinateNode>());  
    rclcpp::shutdown();  
    return 0;  
}