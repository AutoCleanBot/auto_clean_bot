#include "camera_transform.hpp"

CameraTransform::CameraTransform()
{
}

CameraTransform::~CameraTransform()
{
}

void CameraTransform::set_camera_intrinsic_params(const float &fx, const float &fy, const float &cx, const float &cy)
{
    this->_fx = fx;
    this->_fy = fy;
    this->_cx = cx;
    this->_cy = cy;
}

void CameraTransform::set_camera_extrinsic_params(const float &yaw, const float &pitch, const float &roll, const float &tx, const float &ty, const float &tz)
{
    this->_yaw = deg2rad(yaw);
    this->_pitch = deg2rad(pitch);
    this->_roll = deg2rad(roll);
    this->_tx = tx;
    this->_ty = ty;
    this->_tz = tz;
}

Eigen::Vector3f CameraTransform::camera_to_vehicle_coordinate(const float &u, const float &v)
{
    Eigen::Matrix3f transformYaw, transformPitch, transformRoll; // 航向角、俯仰角、横滚角旋转矩阵

    //航向角
    transformYaw << cos(_yaw),-sin(_yaw),0,\
                    sin(_yaw),cos(_yaw),0,\
                    0,0,1;
    //俯仰角
    transformPitch << cos(_pitch),0,sin(_pitch),\
                      0,1,0,\
                      -sin(_pitch),0,cos(_pitch);   
    //横滚角
    transformRoll << 1,0,0,\
                     0,cos(_roll),-sin(_roll),\
                     0,sin(_roll),cos(_roll);

    // 组合旋转矩阵
    Eigen::Matrix3f transform_cam_vehicle = transformRoll * transformPitch * transformYaw;

    // 相机坐标系是左手坐标系，车体坐标系是右手坐标系，需要反转 Z 轴
    Eigen::Matrix3f transform_cam_vehicle_adjusted = transform_cam_vehicle;
    transform_cam_vehicle_adjusted(0, 2) *= -1;  // 反转 Z 轴
    transform_cam_vehicle_adjusted(1, 2) *= -1;
    transform_cam_vehicle_adjusted(2, 0) *= -1;
    transform_cam_vehicle_adjusted(2, 1) *= -1;

    // 平移向量 t_cam_vehicle (假设的相机到车体的平移)
    Eigen::Vector3f t_cam_vehicle(_tx, _ty, _tz);

    // 相机内参矩阵 K
    Eigen::Matrix3f K;
    K << _fx, 0, _cx,
         0, _fy, _cy,
         0, 0, 1;
    // 图像坐标系中的点 (u, v)
    Eigen::Vector3f P_img(u, v, 1.0f);  // 齐次坐标
    // 相机坐标系中的三维点
    Eigen::Vector3f P_cam = K.inverse() * P_img;  // 齐次坐标

    // 相机坐标系转换到车体坐标系
    Eigen::Vector3f P_vehicle = transform_cam_vehicle_adjusted * P_cam + t_cam_vehicle;

    return P_vehicle;
}
