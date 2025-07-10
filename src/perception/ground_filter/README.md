# Ground Filter

## 简介

Ground Filter 是一个 ROS 2 包，用于从点云数据中基于高度滤除地面点。它可以将输入的点云分成地面点云和非地面点云，用于自动驾驶和机器人导航系统中的障碍物检测和避障。

## 功能特点

- 基于高度阈值滤除地面点
- 支持将点云转换到指定坐标系进行处理
- 分别发布地面点云和非地面点云
- 可动态调整高度阈值参数
- 提供可视化工具

## 依赖项

- ROS 2 Foxy
- PCL
- Eigen3
- tf2_ros
- sensor_msgs
- pcl_conversions

## 安装

### 从源码构建

```bash
# 克隆仓库到你的工作空间
cd ~/your_workspace/src
git clone https://github.com/your_username/ground_filter.git

# 安装依赖
sudo apt-get update
sudo apt-get install -y \
  ros-foxy-pcl-ros \
  ros-foxy-tf2-eigen \
  ros-foxy-tf2-geometry-msgs

# 构建
cd ~/your_workspace
colcon build --packages-select ground_filter
source install/setup.bash
```

## 使用方法

### 启动节点

```bash
ros2 launch ground_filter ground_filter.launch.py
```

### 参数配置

可以通过修改 `config/ground_filter.param.yaml` 文件或在启动时传递参数来配置：

```bash
ros2 launch ground_filter ground_filter.launch.py \
  min_height:=-0.1 \
  max_height:=0.1
```

### 主要参数

| 参数名 | 类型 | 默认值 | 描述 |
| ------ | ---- | ------ | ---- |
| update_rate | double | 10.0 | 更新频率 (Hz) |
| min_height | double | -0.2 | 地面高度的最小阈值 (m) |
| max_height | double | 0.2 | 地面高度的最大阈值 (m) |
| base_frame | string | "base_link" | 基准坐标系 |
| target_frame | string | "map" | 目标坐标系 |
| use_sensor_frame | bool | false | 是否使用传感器坐标系 |

### 话题

#### 订阅
- `~/input/points` (sensor_msgs/PointCloud2): 输入点云数据

#### 发布
- `~/output/ground_points` (sensor_msgs/PointCloud2): 地面点云
- `~/output/no_ground_points` (sensor_msgs/PointCloud2): 非地面点云

## 代码结构

```
ground_filter/
├── config/                  # 配置文件
│   ├── ground_filter.param.yaml
│   └── ground_filter_viz.rviz
├── include/ground_filter/
│   ├── ground_filter.hpp    # 主节点类
│   ├── height_filter.hpp    # 高度滤波器类
│   └── visibility_control.hpp
├── launch/                  # 启动文件
│   ├── ground_filter.launch.py
│   ├── ground_filter_viz.launch.py
│   └── ground_filter_demo.launch.py
├── scripts/
│   └── test_ground_filter.py  # 测试脚本
├── src/
│   ├── ground_filter.cpp    # 主节点实现
│   └── height_filter.cpp    # 高度滤波器实现
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 算法说明

地面滤波过程如下：

1. 接收点云数据
2. 如果需要，将点云转换到指定坐标系
3. 使用高度阈值对点云进行分类：
   - 高度小于等于 max_height 的点被视为地面点
   - 高度大于 max_height 的点被视为非地面点
4. 分别发布地面点云和非地面点云

## 可视化

可以使用 RViz 可视化滤波效果：

1. 启动可视化
   ```bash
   ros2 launch ground_filter ground_filter_viz.launch.py
   ```

2. 或者启动完整演示（包括点云发布器、滤波器和可视化）
   ```bash
   ros2 launch ground_filter ground_filter_demo.launch.py
   ```

## 使用示例

### 运行完整演示

我们提供了一个完整的演示启动文件，它会同时启动点云发布器、地面滤波器和 RViz 可视化：

```bash
# 首先确保已经构建并加载环境
cd ~/your_workspace
source install/setup.bash

# 运行演示
ros2 launch ground_filter ground_filter_demo.launch.py
```

这将启动：
1. 点云发布节点 - 生成模拟点云数据
2. 地面滤波器节点 - 将点云分为地面点和非地面点
3. RViz - 可视化原始点云、地面点云和非地面点云

### 调整高度阈值

你可以在运行时调整高度阈值：

```bash
ros2 param set /ground_filter min_height -0.1
ros2 param set /ground_filter max_height 0.1
```

### 与其他传感器集成

要将地面滤波器与自己的传感器集成，只需将传感器发布的点云话题重映射到地面滤波器的输入话题：

```bash
ros2 launch ground_filter ground_filter.launch.py input_points:=/your_sensor/points
```

## 贡献

欢迎提交 Issues 和 Pull Requests。

## 许可证

Apache License 2.0 