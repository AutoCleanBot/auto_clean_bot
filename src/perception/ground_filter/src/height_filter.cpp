#include "ground_filter/height_filter.hpp"

namespace ground_filter {

HeightFilter::HeightFilter(double min_height, double max_height) : min_height_(min_height), max_height_(max_height) {}

void HeightFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_in,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &no_ground_cloud) {
    ground_cloud->clear();
    no_ground_cloud->clear();

    ground_cloud->header = cloud_in->header;
    no_ground_cloud->header = cloud_in->header;

    for (const auto &point : cloud_in->points) {
        // 使用Z坐标作为高度
        if (point.z <= min_height_) {
            ground_cloud->points.push_back(point);
        } else {
            no_ground_cloud->points.push_back(point);
        }
    }

    ground_cloud->width = ground_cloud->points.size();
    ground_cloud->height = 1;
    ground_cloud->is_dense = false;

    no_ground_cloud->width = no_ground_cloud->points.size();
    no_ground_cloud->height = 1;
    no_ground_cloud->is_dense = false;
}

void HeightFilter::setHeightThreshold(double min_height, double max_height) {
    min_height_ = min_height;
    max_height_ = max_height;
}

} // namespace ground_filter