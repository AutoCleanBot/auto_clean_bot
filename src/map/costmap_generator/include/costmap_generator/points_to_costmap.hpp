#ifndef COSTMAP_GENERATOR__POINTS_TO_COSTMAP_HPP_
#define COSTMAP_GENERATOR__POINTS_TO_COSTMAP_HPP_

#include "costmap_generator/grid_map.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

namespace costmap_generator
{
/**
 * @class PointsToCostmap
 * @brief 点云到代价地图转换类
 * 
 * 该类负责将点云数据转换为代价地图，提供了一系列处理点云和生成代价地图的方法
 */
class PointsToCostmap
{
public:
  /**
   * @brief 从点云数据计算代价地图
   * @param maximum_height_thres 点云数据的最大高度阈值
   * @param minimum_height_thres 点云数据的最小高度阈值
   * @param grid_min_value 代价地图的最小值
   * @param grid_max_value 代价地图的最大值
   * @param gridmap 基于网格的代价地图
   * @param gridmap_layer_name 网格地图的图层名称
   * @param in_sensor_points 订阅的点云数据
   * @return 计算得到的代价地图矩阵
   */
  Eigen::MatrixXf makeCostmapFromPoints(
    const double maximum_height_thres, const double minimum_height_thres,
    const double grid_min_value, const double grid_max_value, const GridMap & gridmap,
    const std::string & gridmap_layer_name,
    const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points);

private:
  double grid_length_x_;    ///< 网格X轴长度 (m)
  double grid_length_y_;    ///< 网格Y轴长度 (m)
  double grid_resolution_;  ///< 网格分辨率 (m)
  double grid_position_x_;  ///< 网格中心X坐标 (m)
  double grid_position_y_;  ///< 网格中心Y坐标 (m)

  /**
   * @brief 初始化网格地图参数
   * @param gridmap 要初始化的网格地图对象
   */
  void initGridmapParam(const GridMap & gridmap);

  /**
   * @brief 检查索引是否在网格地图范围内有效
   * @param grid_ind 对应点云的网格索引
   * @return 如果索引有效则返回true
   */
  bool isValidInd(const Eigen::Vector2i & grid_ind);

  /**
   * @brief 从点云获取网格索引
   * @param point 订阅的点云中的一个点
   * @return 网格地图中的索引
   */
  Eigen::Vector2i fetchGridIndexFromPoint(const pcl::PointXYZ & point);

  /**
   * @brief 将点云分配到适当的网格单元
   * @param in_sensor_points 订阅的点云数据
   * @return 网格大小的三维向量，存储了每个网格单元中点的高度
   */
  std::vector<std::vector<std::vector<double>>> assignPoints2GridCell(
    const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points);

  /**
   * @brief 从点云计算代价地图
   * @param maximum_height_thres 点云数据的最大高度阈值
   * @param minimum_height_thres 点云数据的最小高度阈值
   * @param grid_min_value 代价地图的最小值
   * @param grid_max_value 代价地图的最大值
   * @param gridmap 基于网格的代价地图
   * @param gridmap_layer_name 网格地图的图层名称
   * @param grid_vec 网格大小的三维向量，存储了每个网格单元中点的高度
   * @return 计算得到的代价地图矩阵
   */
  Eigen::MatrixXf calculateCostmap(
    const double maximum_height_thres, const double minimum_height_thres,
    const double grid_min_value, const double grid_max_value, const GridMap & gridmap,
    const std::string & gridmap_layer_name,
    const std::vector<std::vector<std::vector<double>>> grid_vec);
};
}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__POINTS_TO_COSTMAP_HPP_ 