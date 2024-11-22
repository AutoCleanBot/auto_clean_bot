#ifndef COORDINATE_TRANSFORM_H  
#define COORDINATE_TRANSFORM_H

#include <iostream>
#include <cmath>
#include "public_define.hpp"
#include <eigen3/Eigen/Dense>

/**
 * @brief   角雷达坐标系和车辆坐标系的转换
 * @param x 角雷达相对于车辆坐标系x轴的偏移量
 * @param y 角雷达相对于车辆坐标系y轴的偏移量  
 * @param theta 角雷达相对于车辆坐标系的旋转角度，为水平方向与y轴的夹角
 */
class CornerRadarTransform {
public:
    CornerRadarTransform(const float &x, const float &y, const float &z, const float &theta, CornerType type = CornerType::LEFT_FRONT);  
    ~CornerRadarTransform();  
    
    //按照角雷达与水平方向的夹角计算
    Eigen::Vector3f transform_point3d(const float &radar_x, const float &radar_y, const float &radar_z);

    //按照角雷达坐标系与车身坐标系的夹角计算
    Eigen::Vector3f transform_coordinate(const float &radar_x, const float &radar_y, const float &radar_z);

private:
    float _tx;
    float _ty; 
    float _tz; 
    float _theta;
    CornerType _corner_type;
};


#endif // COORDINATE_TRANSFORM_H