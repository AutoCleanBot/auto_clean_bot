#ifndef PUBLIC_DEFINE_HPP
#define PUBLIC_DEFINE_HPP

#include <cmath>

#define PI 3.14159265358979323846
#define deg2rad(x) (x * PI / 180.0)
#define rad2deg(x) (x * 180.0 / PI)

// 二维平面中的点
struct Point3D{
    float x;
    float y;
    float z;
};

enum CornerType {
    LEFT_FRONT = 0x00,
    RIGHT_FRONT = 0x01,
    LEFT_BACK = 0x02,
    RIGHT_BACK = 0x03,
    TOP_FRONT = 0x04,
    UNKNOWN = 0xff
};

struct CornerRadar{
    CornerType type;
    float r_theta;
    float tx;
    float ty;
    float tz;
};

#endif // PUBLIC_DEFINE_HPP