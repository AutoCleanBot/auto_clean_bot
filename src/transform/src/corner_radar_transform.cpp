#include "corner_radar_transform.hpp"

CornerRadarTransform::CornerRadarTransform(const float &x, const float &y, const float &z, const float &theta, CornerType type)
{
    //右前
    _tx = x;
    _ty = y;
    _tz = z;
    _theta = theta;    
    _corner_type = type;
}

CornerRadarTransform::~CornerRadarTransform()
{
}
/**
 * @brief 绕z轴的旋转矩阵 theta为垂直于车辆的夹角(锐角)
 */
Eigen::Vector3f CornerRadarTransform::transform_point3d(const float &radar_x, const float &radar_y, const float &radar_z)
{
    //弧度
    float theta_rad;
    switch (_corner_type)
    {
        case CornerType::LEFT_FRONT:
            theta_rad = deg2rad(_theta);
            break;
        case CornerType::RIGHT_FRONT:
            theta_rad = deg2rad(-_theta);
            break;
        case CornerType::LEFT_BACK:
            theta_rad = deg2rad(90 + _theta);
            break;
        case CornerType::RIGHT_BACK:
            theta_rad = deg2rad(270 - _theta);
            break;  
        case CornerType::TOP_FRONT:
            theta_rad = 0;
            break;  
        default:
            break;
    }
    //绕z轴的旋转矩阵
    Eigen::Matrix3f R_z;
    R_z << cos(theta_rad), -sin(theta_rad), 0,\
           sin(theta_rad),  cos(theta_rad), 0,\
           0, 0, 1;
    //雷达点 (x, y, z)
    Eigen::Vector3f radarPoint(radar_x, radar_y, radar_z);
    // 车身坐标系的平移量
    Eigen::Vector3f translation(_tx, _ty, _tz);
    // 将雷达坐标系的点进行旋转变换
    Eigen::Vector3f rotatedPoint = R_z * radarPoint;
    // 平移变换
    Eigen::Vector3f vehiclePoint = rotatedPoint + translation;
    return vehiclePoint;
}

//_theta为绕z逆时针旋转角度
Eigen::Vector3f CornerRadarTransform::transform_coordinate(const float &radar_x, const float &radar_y, const float &radar_z)
{
    float theta_rad = deg2rad(_theta);
    //雷达点 (x, y, z)
    Eigen::Vector3f radarPoint(radar_x, radar_y, radar_z);
    // 车身坐标系的平移量
    Eigen::Vector3f translation(_tx, _ty, _tz);

    //绕z轴的旋转矩阵
    Eigen::Matrix3f R_z;
    R_z << cos(theta_rad), -sin(theta_rad), 0,\
           sin(theta_rad),  cos(theta_rad), 0,\
           0, 0, 1;

    // 将雷达坐标系的点进行旋转变换
    Eigen::Vector3f rotatedPoint = R_z * radarPoint;

    // 平移变换
    Eigen::Vector3f vehiclePoint = rotatedPoint + translation;
    return vehiclePoint;
}
