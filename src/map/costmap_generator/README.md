# Costmap Generator

## 简介

Costmap Generator 是一个 ROS 2 包，用于将点云数据转换为代价地图（Costmap）。代价地图是自动驾驶和机器人导航系统中的重要组件，用于路径规划和避障。

该包参考了 Autoware.universe 中的 costmap_generator 模块，但进行了简化，专注于从点云数据生成代价地图。

## 功能特点

- 将点云数据转换为代价地图
- 支持动态更新代价地图中心（跟随车辆位置）
- 发布 GridMap 和 OccupancyGrid 格式的代价地图
- 可配置的代价地图参数（分辨率、大小、高度阈值等）

## 依赖项

- ROS 2 Foxy
- PCL
- Eigen3
- grid_map_msgs
- nav_msgs
- tf2_ros
- sensor_msgs

## 安装

### 从源码构建

```bash
# 克隆仓库到你的工作空间
cd ~/your_workspace/src
git clone https://github.com/your_username/costmap_generator.git

# 安装依赖
sudo apt-get update
sudo apt-get install -y \
  ros-foxy-pcl-ros \
  ros-foxy-tf2-eigen \
  ros-foxy-tf2-geometry-msgs

# 构建
cd ~/your_workspace
colcon build --packages-select costmap_generator
source install/setup.bash
```

## 使用方法

### 启动节点

```bash
ros2 launch costmap_generator costmap_generator.launch.py
```

### 参数配置

可以通过修改 `config/costmap_generator.param.yaml` 文件或在启动时传递参数来配置：

```bash
ros2 launch costmap_generator costmap_generator.launch.py \
  grid_resolution:=0.1 \
  grid_length_x:=100.0 \
  grid_length_y:=100.0
```

### 主要参数

| 参数名 | 类型 | 默认值 | 描述 |
| ------ | ---- | ------ | ---- |
| update_rate | double | 10.0 | 更新频率 (Hz) |
| grid_resolution | double | 0.2 | 栅格分辨率 (m) |
| grid_length_x | double | 50.0 | 代价地图 X 轴长度 (m) |
| grid_length_y | double | 50.0 | 代价地图 Y 轴长度 (m) |
| grid_position_x | double | 0.0 | 代价地图中心 X 坐标 (m) |
| grid_position_y | double | 0.0 | 代价地图中心 Y 坐标 (m) |
| maximum_height_thres | double | 2.0 | 最大高度阈值 (m) |
| minimum_height_thres | double | 0.2 | 最小高度阈值 (m) |
| grid_min_value | double | 0.0 | 代价地图最小值 |
| grid_max_value | double | 1.0 | 代价地图最大值 |
| costmap_frame | string | "map" | 代价地图坐标系 |
| vehicle_frame | string | "base_link" | 车辆坐标系 |

### 话题

#### 订阅
- `~/input/points` (sensor_msgs/PointCloud2): 输入点云数据

#### 发布
- `~/output/grid_map` (grid_map_msgs/GridMap): 栅格地图格式的代价地图
- `~/output/occupancy_grid` (nav_msgs/OccupancyGrid): 占用栅格格式的代价地图

## 代码结构

```
costmap_generator/
├── config/                  # 配置文件
│   └── costmap_generator.param.yaml
├── include/costmap_generator/
│   ├── costmap_generator.hpp    # 主节点类
│   ├── grid_map.hpp            # 自定义 GridMap 类
│   └── points_to_costmap.hpp    # 点云到代价地图转换类
├── launch/                  # 启动文件
│   └── costmap_generator.launch.py
├── src/
│   ├── costmap_generator.cpp    # 主节点实现
│   ├── costmap_node.cpp        # 节点入口
│   ├── grid_map.cpp            # GridMap 类实现
│   └── points_to_costmap.cpp    # 点云转换实现
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 算法说明

代价地图生成过程如下：

1. 接收点云数据
2. 将点云转换到代价地图坐标系
3. 根据点云高度信息计算每个栅格的代价值
   - 高度在 minimum_height_thres 和 maximum_height_thres 之间的点被视为障碍物
   - 代价值根据点的高度线性插值
4. 更新代价地图
5. 发布 GridMap 和 OccupancyGrid 格式的代价地图

## 可视化

可以使用 RViz 可视化代价地图：

1. 打开 RViz
2. 添加 OccupancyGrid 显示
3. 设置话题为 `/costmap_generator/output/occupancy_grid`

## 使用示例

### 运行完整演示

我们提供了一个完整的演示启动文件，它会同时启动代价地图生成器和 RViz 可视化：

```bash
# 首先确保已经构建并加载环境
cd ~/your_workspace
source install/setup.bash

# 运行演示
ros2 launch costmap_generator costmap_demo.launch.py
```

这将启动：
1. 点云发布节点 - 生成模拟点云数据
2. 代价地图生成器节点 - 将点云转换为代价地图
3. RViz - 可视化点云和代价地图

### 测试脚本

我们提供了一个测试脚本 `scripts/test_costmap_generator.py`，它会生成模拟点云数据并发布到 `/points_no_ground` 话题。这个脚本会创建一个包含三个障碍物的简单场景：

- 在 (2, 2) 位置有一个高度为 1m 的障碍物
- 在 (-2, -2) 位置有一个高度为 0.5m 的障碍物
- 在 (0, 0) 位置有一个高度为 1.5m 的障碍物

你可以单独运行这个脚本：

```bash
ros2 run costmap_generator test_costmap_generator.py
```

然后在另一个终端中启动代价地图生成器：

```bash
ros2 launch costmap_generator costmap_generator.launch.py
```

### 集成到自己的项目

要将代价地图生成器集成到自己的项目中，只需确保发布正确格式的点云数据到相应的话题，并在启动文件中配置适当的参数。

例如，如果你的点云数据发布在 `/lidar/points` 话题上，可以使用以下命令启动代价地图生成器：

```bash
ros2 launch costmap_generator costmap_generator.launch.py \
  grid_resolution:=0.1 \
  grid_length_x:=50.0 \
  grid_length_y:=50.0
```

并添加以下重映射：

```
--remap ~/input/points:=/lidar/points
```

## 贡献

欢迎提交 Issues 和 Pull Requests。

## 许可证

Apache License 2.0 