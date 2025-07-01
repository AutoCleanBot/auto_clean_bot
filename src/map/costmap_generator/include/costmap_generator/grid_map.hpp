#ifndef COSTMAP_GENERATOR__GRID_MAP_HPP_
#define COSTMAP_GENERATOR__GRID_MAP_HPP_

#include <Eigen/Core>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace costmap_generator
{

class GridMap
{
public:
  GridMap() = default;
  explicit GridMap(const std::vector<std::string> & layers);

  void setFrameId(const std::string & frame_id);
  void setGeometry(
    const Eigen::Vector2d & length, const double resolution, const Eigen::Vector2d & position);
  void setTimestamp(const uint64_t timestamp);
  void move(const Eigen::Vector2d & position);

  const std::string & getFrameId() const;
  const Eigen::Vector2d & getLength() const;
  double getResolution() const;
  const Eigen::Vector2d & getPosition() const;
  uint64_t getTimestamp() const;
  const std::vector<std::string> & getLayers() const;

  Eigen::MatrixXf & operator[](const std::string & layer);
  const Eigen::MatrixXf & operator[](const std::string & layer) const;

  void toMessage(grid_map_msgs::msg::GridMap & message) const;
  void toOccupancyGrid(
    const std::string & layer, const double min_value, const double max_value,
    nav_msgs::msg::OccupancyGrid & occupancy_grid) const;

private:
  std::string frame_id_;
  Eigen::Vector2d length_;
  double resolution_;
  Eigen::Vector2d position_;
  uint64_t timestamp_;
  std::vector<std::string> layers_;
  std::unordered_map<std::string, Eigen::MatrixXf> data_;
};

}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__GRID_MAP_HPP_ 