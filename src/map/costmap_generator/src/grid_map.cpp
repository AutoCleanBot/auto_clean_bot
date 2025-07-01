#include "costmap_generator/grid_map.hpp"

#include <algorithm>
#include <cmath>

namespace costmap_generator {

GridMap::GridMap(const std::vector<std::string> &layers) : layers_(layers) {
    for (const auto &layer : layers_) {
        data_[layer] = Eigen::MatrixXf();
    }
}

void GridMap::setFrameId(const std::string &frame_id) { frame_id_ = frame_id; }

void GridMap::setGeometry(const Eigen::Vector2d &length, const double resolution, const Eigen::Vector2d &position) {
    length_ = length;
    resolution_ = resolution;
    position_ = position;

    // Initialize all layers with zero matrices
    const int rows = std::ceil(length_.x() / resolution_);
    const int cols = std::ceil(length_.y() / resolution_);

    for (const auto &layer : layers_) {
        data_[layer] = Eigen::MatrixXf::Zero(rows, cols);
    }
}

void GridMap::setTimestamp(const uint64_t timestamp) { timestamp_ = timestamp; }

void GridMap::move(const Eigen::Vector2d &position) {
    position_ = position;
    // Note: In a real implementation, we would also need to shift the data matrices
}

const std::string &GridMap::getFrameId() const { return frame_id_; }

const Eigen::Vector2d &GridMap::getLength() const { return length_; }

double GridMap::getResolution() const { return resolution_; }

const Eigen::Vector2d &GridMap::getPosition() const { return position_; }

uint64_t GridMap::getTimestamp() const { return timestamp_; }

const std::vector<std::string> &GridMap::getLayers() const { return layers_; }

Eigen::MatrixXf &GridMap::operator[](const std::string &layer) { return data_[layer]; }

const Eigen::MatrixXf &GridMap::operator[](const std::string &layer) const { return data_.at(layer); }

void GridMap::toMessage(grid_map_msgs::msg::GridMap &message) const {
    message.header.frame_id = frame_id_;
    message.header.stamp.sec = timestamp_ / 1000000000ULL;
    message.header.stamp.nanosec = timestamp_ % 1000000000ULL;
    message.info.resolution = resolution_;
    message.info.length_x = length_.x();
    message.info.length_y = length_.y();
    message.info.pose.position.x = position_.x();
    message.info.pose.position.y = position_.y();
    message.info.pose.position.z = 0.0;
    message.info.pose.orientation.w = 1.0;
    message.info.pose.orientation.x = 0.0;
    message.info.pose.orientation.y = 0.0;
    message.info.pose.orientation.z = 0.0;

    message.layers = layers_;
    message.basic_layers = layers_;

    const int rows = data_.at(layers_[0]).rows();
    const int cols = data_.at(layers_[0]).cols();

    message.data.clear();
    message.data.resize(layers_.size());

    for (size_t i = 0; i < layers_.size(); ++i) {
        const auto &layer = layers_[i];
        const auto &matrix = data_.at(layer);

        message.data[i].layout.dim.resize(2);
        message.data[i].layout.dim[0].label = "rows";
        message.data[i].layout.dim[0].size = rows;
        message.data[i].layout.dim[0].stride = rows * cols;
        message.data[i].layout.dim[1].label = "cols";
        message.data[i].layout.dim[1].size = cols;
        message.data[i].layout.dim[1].stride = cols;

        message.data[i].data.resize(rows * cols);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                message.data[i].data[r * cols + c] = matrix(r, c);
            }
        }
    }
}

void GridMap::toOccupancyGrid(const std::string &layer, const double min_value, const double max_value,
                              nav_msgs::msg::OccupancyGrid &occupancy_grid) const {
    const auto &matrix = data_.at(layer);
    const int rows = matrix.rows();
    const int cols = matrix.cols();

    occupancy_grid.header.frame_id = frame_id_;
    occupancy_grid.header.stamp.sec = timestamp_ / 1000000000ULL;
    occupancy_grid.header.stamp.nanosec = timestamp_ % 1000000000ULL;
    occupancy_grid.info.resolution = resolution_;
    occupancy_grid.info.width = cols;
    occupancy_grid.info.height = rows;
    occupancy_grid.info.origin.position.x = position_.x() - length_.x() / 2.0;
    occupancy_grid.info.origin.position.y = position_.y() - length_.y() / 2.0;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.info.origin.orientation.x = 0.0;
    occupancy_grid.info.origin.orientation.y = 0.0;
    occupancy_grid.info.origin.orientation.z = 0.0;

    occupancy_grid.data.resize(rows * cols);

    const double range = max_value - min_value;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const double value = matrix(r, c);
            if (std::isnan(value)) {
                occupancy_grid.data[r * cols + c] = -1; // Unknown
            } else {
                const double normalized = std::min(std::max((value - min_value) / range, 0.0), 1.0);
                occupancy_grid.data[r * cols + c] = static_cast<int8_t>(normalized * 100.0);
            }
        }
    }
}

} // namespace costmap_generator