#ifndef GROUND_FILTER__HEIGHT_FILTER_HPP_
#define GROUND_FILTER__HEIGHT_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace ground_filter
{

class HeightFilter
{
public:
  /**
   * @brief 构造函数
   * @param min_height 地面高度的最小阈值，低于此值的点被视为地面点
   * @param max_height 地面高度的最大阈值，介于min_height和max_height之间的点可能是地面点
   */
  HeightFilter(double min_height, double max_height);

  /**
   * @brief 根据高度将点云分为地面点和非地面点
   * @param cloud_in 输入点云
   * @param ground_cloud 输出的地面点云
   * @param no_ground_cloud 输出的非地面点云
   */
  void filter(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & ground_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & no_ground_cloud);

  /**
   * @brief 设置高度阈值
   * @param min_height 地面高度的最小阈值
   * @param max_height 地面高度的最大阈值
   */
  void setHeightThreshold(double min_height, double max_height);

private:
  double min_height_;  // 地面高度的最小阈值
  double max_height_;  // 地面高度的最大阈值
};

}  // namespace ground_filter

#endif  // GROUND_FILTER__HEIGHT_FILTER_HPP_ 