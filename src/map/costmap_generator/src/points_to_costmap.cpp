#include "costmap_generator/points_to_costmap.hpp"

namespace costmap_generator {

Eigen::MatrixXf PointsToCostmap::makeCostmapFromPoints(const double maximum_height_thres,
                                                       const double minimum_height_thres, const double grid_min_value,
                                                       const double grid_max_value, const GridMap &gridmap,
                                                       const std::string &gridmap_layer_name,
                                                       const pcl::PointCloud<pcl::PointXYZ> &in_sensor_points) {
    initGridmapParam(gridmap);
    std::vector<std::vector<std::vector<double>>> grid_vec = assignPoints2GridCell(in_sensor_points);
    return calculateCostmap(maximum_height_thres, minimum_height_thres, grid_min_value, grid_max_value, gridmap,
                            gridmap_layer_name, grid_vec);
}

void PointsToCostmap::initGridmapParam(const GridMap &gridmap) {
    grid_length_x_ = gridmap.getLength().x();
    grid_length_y_ = gridmap.getLength().y();
    grid_resolution_ = gridmap.getResolution();
    grid_position_x_ = gridmap.getPosition().x();
    grid_position_y_ = gridmap.getPosition().y();
}

bool PointsToCostmap::isValidInd(const Eigen::Vector2i &grid_ind) {
    const int x_grid_ind = grid_ind.x();
    const int y_grid_ind = grid_ind.y();
    const int x_grid_size = std::ceil(grid_length_x_ / grid_resolution_);
    const int y_grid_size = std::ceil(grid_length_y_ / grid_resolution_);

    if (x_grid_ind < 0 || x_grid_ind >= x_grid_size || y_grid_ind < 0 || y_grid_ind >= y_grid_size) {
        return false;
    }
    return true;
}

Eigen::Vector2i PointsToCostmap::fetchGridIndexFromPoint(const pcl::PointXYZ &point) {
    const double origin_x = grid_position_x_ - grid_length_x_ / 2.0;
    const double origin_y = grid_position_y_ - grid_length_y_ / 2.0;
    const int x = std::floor((point.x - origin_x) / grid_resolution_);
    const int y = std::floor((point.y - origin_y) / grid_resolution_);
    return Eigen::Vector2i(x, y);
}

std::vector<std::vector<std::vector<double>>>
PointsToCostmap::assignPoints2GridCell(const pcl::PointCloud<pcl::PointXYZ> &in_sensor_points) {
    const int x_grid_size = std::ceil(grid_length_x_ / grid_resolution_);
    const int y_grid_size = std::ceil(grid_length_y_ / grid_resolution_);
    std::vector<std::vector<std::vector<double>>> grid_vec(
        x_grid_size, std::vector<std::vector<double>>(y_grid_size, std::vector<double>()));

    for (const auto &point : in_sensor_points) {
        Eigen::Vector2i grid_ind = fetchGridIndexFromPoint(point);
        if (isValidInd(grid_ind)) {
            grid_vec[grid_ind.x()][grid_ind.y()].push_back(point.z);
        }
    }
    return grid_vec;
}

Eigen::MatrixXf PointsToCostmap::calculateCostmap(const double maximum_height_thres, const double minimum_height_thres,
                                                  const double grid_min_value, const double grid_max_value,
                                                  const GridMap &gridmap, const std::string &gridmap_layer_name,
                                                  const std::vector<std::vector<std::vector<double>>> grid_vec) {
    Eigen::MatrixXf costmap = gridmap[gridmap_layer_name];
    const int x_grid_size = std::ceil(grid_length_x_ / grid_resolution_);
    const int y_grid_size = std::ceil(grid_length_y_ / grid_resolution_);

    for (int x_ind = 0; x_ind < x_grid_size; ++x_ind) {
        for (int y_ind = 0; y_ind < y_grid_size; ++y_ind) {
            const auto &z_vec = grid_vec[x_ind][y_ind];
            if (z_vec.empty()) {
                continue;
            }

            double max_z_value = *std::max_element(z_vec.begin(), z_vec.end());
            double min_z_value = *std::min_element(z_vec.begin(), z_vec.end());

            if (maximum_height_thres > max_z_value && max_z_value > minimum_height_thres &&
                min_z_value > minimum_height_thres) {
                const double ratio =
                    std::min(1.0, (max_z_value - minimum_height_thres) / (maximum_height_thres - minimum_height_thres));
                const double cost = ratio * (grid_max_value - grid_min_value) + grid_min_value;
                costmap(x_ind, y_ind) = cost;
            }
        }
    }
    return costmap;
}

} // namespace costmap_generator