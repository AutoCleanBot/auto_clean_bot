#include "lidar_transform.hpp"

LidarTransform::LidarTransform(double yaw, double pitch, double roll, double x, double y, double z)
{
    _yaw = deg2rad(yaw);
    _pitch = deg2rad(pitch);
    _roll = deg2rad(roll);
    _x = x;
    _y = y;
    _z = z;
}   

LidarTransform::~LidarTransform()
{
}

void LidarTransform::transform_point3d(pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr carCloud)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transformYaw;
    Eigen::Matrix4f transformPitch;
    Eigen::Matrix4f transformRoll;
    
    //航向角
    transformYaw << cos(_yaw),-sin(_yaw),0,0,\
                    sin(_yaw),cos(_yaw),0,0,\
                    0,0,1,0,\
                    0,0,0,1;
    //俯仰角
    transformPitch << cos(_pitch),0,sin(_pitch),0,\
                      0,1,0,0,\
                      -sin(_pitch),0,cos(_pitch),0,\
                      0,0,0,1;    
    //横滚角
    transformRoll << 1,0,0,0,\
                     0,cos(_roll),-sin(_roll),0,\
                     0,sin(_roll),cos(_roll),0,\
                     0,0,0,1;
            
    //旋转矩阵
    transform = transformRoll*transformPitch*transformYaw;
    
    //平移矩阵
    transform(0,3) = _x;
    transform(1,3) = _y;
    transform(2,3) = _z;
    
    //坐标转换
    pcl::transformPointCloud(*lidarCloud,*carCloud,transform);
}
