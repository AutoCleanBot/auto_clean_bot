#include "costmap_generator/points_to_costmap.hpp"

namespace costmap_generator {

/**
 * @brief 从点云数据生成代价地图的主要接口函数
 *
 * 该函数是点云到代价地图转换的主要入口点，它协调整个转换过程：
 * 1. 初始化网格地图参数
 * 2. 将点云分配到网格单元
 * 3. 计算最终的代价地图
 *
 * @param maximum_height_thres 点云高度的最大阈值，超过此高度的点将被忽略
 * @param minimum_height_thres 点云高度的最小阈值，低于此高度的点将被忽略
 * @param grid_min_value 代价地图中的最小代价值
 * @param grid_max_value 代价地图中的最大代价值
 * @param gridmap 输入的网格地图对象，提供地图的几何参数
 * @param gridmap_layer_name 要处理的网格地图图层名称
 * @param in_sensor_points 输入的点云数据
 * @return 生成的代价地图矩阵
 */
Eigen::MatrixXf PointsToCostmap::makeCostmapFromPoints(const double maximum_height_thres,
                                                       const double minimum_height_thres, const double grid_min_value,
                                                       const double grid_max_value, const GridMap &gridmap,
                                                       const std::string &gridmap_layer_name,
                                                       const pcl::PointCloud<pcl::PointXYZ> &in_sensor_points) {
    // 初始化网格地图参数
    initGridmapParam(gridmap);
    // 将点云分配到对应的网格单元中
    std::vector<std::vector<std::vector<double>>> grid_vec = assignPoints2GridCell(in_sensor_points);
    // 基于分配的点云数据计算代价地图
    return calculateCostmap(maximum_height_thres, minimum_height_thres, grid_min_value, grid_max_value, gridmap,
                            gridmap_layer_name, grid_vec);
}

/**
 * @brief 从网格地图对象初始化内部参数
 *
 * 该函数从输入的网格地图对象中提取关键参数并存储到类的成员变量中，
 * 这些参数将在后续的点云处理和代价地图计算中使用。
 *
 * @param gridmap 输入的网格地图对象，包含地图的几何信息
 */
void PointsToCostmap::initGridmapParam(const GridMap &gridmap) {
    grid_length_x_ = gridmap.getLength().x();     // 获取网格地图X轴方向的长度
    grid_length_y_ = gridmap.getLength().y();     // 获取网格地图Y轴方向的长度
    grid_resolution_ = gridmap.getResolution();   // 获取网格地图的分辨率（每个网格的大小）
    grid_position_x_ = gridmap.getPosition().x(); // 获取网格地图中心点的X坐标
    grid_position_y_ = gridmap.getPosition().y(); // 获取网格地图中心点的Y坐标
}

/**
 * @brief 检查网格索引是否在有效范围内
 *
 * 该函数验证给定的网格索引是否在网格地图的有效范围内。
 * 网格地图的大小由地图长度和分辨率决定。
 *
 * @param grid_ind 要检查的网格索引（x, y坐标）
 * @return 如果索引在有效范围内返回true，否则返回false
 */
bool PointsToCostmap::isValidInd(const Eigen::Vector2i &grid_ind) {
    const int x_grid_ind = grid_ind.x(); // 获取X轴网格索引
    const int y_grid_ind = grid_ind.y(); // 获取Y轴网格索引
    // 根据地图长度和分辨率计算网格大小
    const int x_grid_size = std::ceil(grid_length_x_ / grid_resolution_);
    const int y_grid_size = std::ceil(grid_length_y_ / grid_resolution_);

    // 检查索引是否在有效范围内（0 <= index < grid_size）
    if (x_grid_ind < 0 || x_grid_ind >= x_grid_size || y_grid_ind < 0 || y_grid_ind >= y_grid_size) {
        return false;
    }
    return true;
}

/**
 * @brief 从点云中的点计算对应的网格索引
 *
 * 该函数将3D点云中的点坐标转换为2D网格地图中的索引。
 * 转换过程：
 * 1. 计算网格地图的原点坐标（左下角）
 * 2. 将点的世界坐标转换为相对于原点的坐标
 * 3. 除以分辨率得到网格索引
 *
 * @param point 点云中的一个点，包含x, y, z坐标
 * @return 对应的网格索引（x, y）
 */
Eigen::Vector2i PointsToCostmap::fetchGridIndexFromPoint(const pcl::PointXYZ &point) {
    // 计算网格地图的原点坐标（网格地图左下角的世界坐标）
    const double origin_x = grid_position_x_ - grid_length_x_ / 2.0;
    const double origin_y = grid_position_y_ - grid_length_y_ / 2.0;
    // 将点的世界坐标转换为网格索引
    const int x = std::floor((point.x - origin_x) / grid_resolution_);
    const int y = std::floor((point.y - origin_y) / grid_resolution_);

    // 调试输出：显示前10个点的转换过程
    static int debug_count = 0;
    if (debug_count < 10) {
        std::cout << "Point (" << point.x << ", " << point.y << ", " << point.z << ") -> Grid index (" << x << ", " << y
                  << ")" << std::endl;
        std::cout << "  Origin: (" << origin_x << ", " << origin_y << ")" << std::endl;
        std::cout << "  Grid center: (" << grid_position_x_ << ", " << grid_position_y_ << ")" << std::endl;
        std::cout << "  Grid resolution: " << grid_resolution_ << std::endl;
        debug_count++;
    }

    return Eigen::Vector2i(x, y);
}

/**
 * @brief 将点云数据分配到对应的网格单元中
 *
 * 该函数创建一个三维向量结构来存储点云数据：
 * - 第一维：X轴网格索引
 * - 第二维：Y轴网格索引
 * - 第三维：该网格单元中所有点的Z坐标值
 *
 * 处理过程：
 * 1. 创建网格大小的三维向量结构
 * 2. 遍历所有点云数据
 * 3. 计算每个点对应的网格索引
 * 4. 将有效点的Z坐标存储到对应网格单元中
 *
 * @param in_sensor_points 输入的点云数据
 * @return 三维向量，存储每个网格单元中点的Z坐标值
 */
std::vector<std::vector<std::vector<double>>>
PointsToCostmap::assignPoints2GridCell(const pcl::PointCloud<pcl::PointXYZ> &in_sensor_points) {
    // 根据地图长度和分辨率计算网格大小
    const int x_grid_size = std::ceil(grid_length_x_ / grid_resolution_);
    const int y_grid_size = std::ceil(grid_length_y_ / grid_resolution_);
    // 创建三维向量结构：grid_vec[x][y] = vector<double>存储该网格单元的所有Z值
    std::vector<std::vector<std::vector<double>>> grid_vec(
        x_grid_size, std::vector<std::vector<double>>(y_grid_size, std::vector<double>()));

    // 输出网格参数信息用于调试
    std::cout << "Grid dimensions: " << x_grid_size << " x " << y_grid_size << std::endl;
    std::cout << "Grid resolution: " << grid_resolution_ << std::endl;
    std::cout << "Grid position: (" << grid_position_x_ << ", " << grid_position_y_ << ")" << std::endl;
    std::cout << "Grid length: " << grid_length_x_ << " x " << grid_length_y_ << std::endl;

    int valid_points = 0;   // 有效点计数器
    int invalid_points = 0; // 无效点计数器

    // 遍历所有点云数据
    for (const auto &point : in_sensor_points) {
        // 计算点对应的网格索引
        Eigen::Vector2i grid_ind = fetchGridIndexFromPoint(point);
        // 检查索引是否有效
        if (isValidInd(grid_ind)) {
            // 将点的Z坐标存储到对应的网格单元中
            grid_vec[grid_ind.x()][grid_ind.y()].push_back(point.z);
            valid_points++;
        } else {
            invalid_points++;
        }
    }

    // 输出点分配统计信息
    std::cout << "Points assigned to grid cells: " << valid_points << " valid, " << invalid_points << " invalid"
              << std::endl;

    // 统计并输出非空网格单元的数量
    int non_empty_cells = 0;
    for (int x = 0; x < x_grid_size; ++x) {
        for (int y = 0; y < y_grid_size; ++y) {
            if (!grid_vec[x][y].empty()) {
                non_empty_cells++;
            }
        }
    }
    std::cout << "Non-empty grid cells: " << non_empty_cells << " out of " << x_grid_size * y_grid_size << std::endl;

    return grid_vec;
}

/**
 * @brief 基于分配的点云数据计算代价地图
 *
 * 该函数是代价地图生成的核心算法，它根据每个网格单元中点的高度信息
 * 计算该单元的代价值。计算逻辑：
 * 1. 对于每个网格单元，找出其中点的最大和最小Z值
 * 2. 检查这些Z值是否在指定的高度阈值范围内
 * 3. 如果满足条件，根据高度比例计算代价值
 * 4. 代价值在grid_min_value和grid_max_value之间线性插值
 *
 * @param maximum_height_thres 点云高度的最大阈值
 * @param minimum_height_thres 点云高度的最小阈值
 * @param grid_min_value 代价地图的最小代价值
 * @param grid_max_value 代价地图的最大代价值
 * @param gridmap 输入的网格地图对象
 * @param gridmap_layer_name 要处理的网格地图图层名称
 * @param grid_vec 包含每个网格单元点云数据的三维向量
 * @return 计算得到的代价地图矩阵
 */
Eigen::MatrixXf PointsToCostmap::calculateCostmap(const double maximum_height_thres, const double minimum_height_thres,
                                                  const double grid_min_value, const double grid_max_value,
                                                  const GridMap &gridmap, const std::string &gridmap_layer_name,
                                                  const std::vector<std::vector<std::vector<double>>> grid_vec) {
    // 从网格地图获取指定图层作为代价地图的基础
    Eigen::MatrixXf costmap = gridmap[gridmap_layer_name];
    // 计算网格大小
    const int x_grid_size = std::ceil(grid_length_x_ / grid_resolution_);
    const int y_grid_size = std::ceil(grid_length_y_ / grid_resolution_);

    // 遍历所有网格单元
    for (int x_ind = 0; x_ind < x_grid_size; ++x_ind) {
        for (int y_ind = 0; y_ind < y_grid_size; ++y_ind) {
            // 获取当前网格单元中的所有Z值
            const auto &z_vec = grid_vec[x_ind][y_ind];
            // 如果网格单元为空，跳过处理
            if (z_vec.empty()) {
                continue;
            }

            // 找出该网格单元中点的最大和最小Z值
            double max_z_value = *std::max_element(z_vec.begin(), z_vec.end());
            double min_z_value = *std::min_element(z_vec.begin(), z_vec.end());

            // 检查Z值是否在有效范围内：
            // 1. 最大Z值小于最大阈值
            // 2. 最大Z值大于最小阈值
            // 3. 最小Z值大于最小阈值
            if (maximum_height_thres > max_z_value && max_z_value > minimum_height_thres &&
                min_z_value > minimum_height_thres) {
                // 计算高度比例（0-1之间）
                const double ratio =
                    std::min(1.0, (max_z_value - minimum_height_thres) / (maximum_height_thres - minimum_height_thres));
                // 根据比例在最小值和最大值之间线性插值计算代价
                const double cost = ratio * (grid_max_value - grid_min_value) + grid_min_value;
                // 设置代价地图中对应位置的代价值
                costmap(x_ind, y_ind) = cost;
            }
        }
    }
    return costmap;
}

} // namespace costmap_generator