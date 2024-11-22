#ifndef LIDAR_TRANSFORM_H
#define LIDAR_TRANSFORM_H

#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <cmath>
#include "public_define.hpp"

/**
 * @brief   激光雷达坐标系和车辆坐标系的转换
 * @param yaw 激光雷达相对于车辆坐标系的偏航角 绕z轴旋转
 * @param pitch 激光雷达相对于车辆坐标系的俯仰角 绕y轴旋转
 * @param roll 激光雷达相对于车辆坐标系的滚转角 绕x轴旋转
 * @param x 激光雷达相对于车辆坐标系x轴的偏移量  
 * @param y 激光雷达相对于车辆坐标系y轴的偏移量  
 * @param z 激光雷达相对于车辆坐标系z轴的偏移量
 */
class LidarTransform {
public:
    LidarTransform(double yaw, double pitch, double roll, double x, double y, double z);  
    ~LidarTransform();  
    
    void transform_point3d(pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr carCloud);

private:
    float _yaw;
    float _pitch; 
    float _roll; 
    float _x;
    float _y;
    float _z;
};

#endif  // LIDAR_TRANSFORM_H