#include <tf2_eigen/tf2_eigen.h>

void fix_transform() {
    geometry_msgs::msg::TransformStamped transform;
    Eigen::Affine3d transform_eigen;
    
    // 正确的转换方法
    Eigen::Vector3d translation(transform.transform.translation.x, 
                               transform.transform.translation.y, 
                               transform.transform.translation.z);
    Eigen::Quaterniond rotation(transform.transform.rotation.w,
                              transform.transform.rotation.x,
                              transform.transform.rotation.y,
                              transform.transform.rotation.z);
    transform_eigen = Eigen::Translation3d(translation) * rotation;
}
