#ifndef CAMERA_TRANSFORM_HPP
#define CAMERA_TRANSFORM_HPP

#include <Eigen/Dense>
#include <iostream>
#include "public_define.hpp"

class CameraTransform
{
public:
    CameraTransform();
    ~CameraTransform();

    void set_camera_intrinsic_params(const float &fx, const float &fy, const float &cx, const float &cy);
    void set_camera_extrinsic_params(const float &yaw, const float &pitch, const float &roll, const float &tx, const float &ty, const float &tz);

    Eigen::Vector3f camera_to_vehicle_coordinate(const float &u, const float &v);

private:
    float _fx, _fy, _cx, _cy;
    float _yaw, _pitch, _roll, _tx, _ty, _tz;
};

#endif // CAMERA_TRANSFORM_HPP
