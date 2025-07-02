# PointCloud Preprocess

这个ROS 2包提供了激光点云的预处理功能，包括坐标转换、车辆点云过滤和降采样。

## 功能

- 将点云从一个坐标系转换到另一个坐标系
- 支持动态配置输入和输出话题
- 支持使用传感器原始坐标系或指定的输入坐标系
- 支持使用点云时间戳对应的变换或最新的变换
- 车辆内部点云过滤（可配置车辆尺寸）
- 体素网格降采样（可配置是否启用和体素大小）

## 参数

- `update_rate`: 参数更新频率 (Hz)
- `input_frame`: 输入点云坐标系，空字符串表示使用sensor_frame
- `output_frame`: 输出点云坐标系
- `use_sensor_frame`: 是否使用传感器坐标系作为输入坐标系
- `timeout`: TF查询超时时间 (秒)
- `use_latest_transforms`: 是否使用最新的变换而不是点云时间戳对应的变换
- `input_topic`: 输入点云话题
- `output_topic`: 输出点云话题

### 车辆过滤参数

- `filter_vehicle_points`: 是否启用车辆点云过滤
- `vehicle_front_length`: 车辆前部长度 (米)
- `vehicle_back_length`: 车辆后部长度 (米)
- `vehicle_width`: 车辆总宽度 (米)
- `vehicle_height`: 车辆总高度 (米)
- `vehicle_right_width`: 车辆右侧宽度 (米)
- `vehicle_left_width`: 车辆左侧宽度 (米)
- `vehicle_top_height`: 车辆顶部高度 (米)
- `vehicle_bottom_height`: 车辆底部高度 (米)
- `vehicle_x_offset`: 车辆X轴偏移 (米)
- `vehicle_y_offset`: 车辆Y轴偏移 (米)
- `vehicle_z_offset`: 车辆Z轴偏移 (米)
- `vehicle_length_margin`: 车辆长度边界扩展 (米)
- `vehicle_width_margin`: 车辆宽度边界扩展 (米)
- `vehicle_height_margin`: 车辆高度边界扩展 (米)

### 降采样参数

- `enable_downsampling`: 是否启用降采样
- `voxel_leaf_size`: 体素网格叶子大小 (米)，值越小保留的点越多，处理时间越长

## 用法

### 启动节点

```bash
ros2 launch pointcloud_preprocess pointcloud_transformer.launch.py
```

### 自定义参数启动

```bash
ros2 launch pointcloud_preprocess pointcloud_transformer.launch.py input_topic:=/lidar/points output_topic:=/lidar/transformed_points output_frame:=odom
```

### 运行时修改参数

```bash
# 修改话题和坐标系
ros2 param set /pointcloud_transformer input_topic /new_input_topic
ros2 param set /pointcloud_transformer output_topic /new_output_topic
ros2 param set /pointcloud_transformer output_frame map

# 启用/禁用车辆过滤
ros2 param set /pointcloud_transformer filter_vehicle_points true

# 启用/禁用降采样
ros2 param set /pointcloud_transformer enable_downsampling true
ros2 param set /pointcloud_transformer voxel_leaf_size 0.05
```

## 处理流程

点云数据的处理顺序如下：

1. **坐标转换**：将点云从输入坐标系转换到输出坐标系
2. **车辆过滤**：移除车辆内部的点云（如果启用）
3. **降采样**：使用体素网格进行降采样（如果启用）
4. **发布**：发布处理后的点云

## 降采样说明

降采样使用PCL的体素网格滤波器（VoxelGrid），将点云空间划分为立方体网格，每个网格内的所有点用一个代表点替换。

- **优点**：显著减少点云数量，提高后续处理速度
- **缺点**：会丢失一些细节信息
- **建议**：
  - 对于实时应用，建议启用降采样
  - `voxel_leaf_size` 设置为 0.05-0.2 米通常效果较好
  - 值越小保留细节越多，但处理时间越长

## 依赖项

- rclcpp
- sensor_msgs
- pcl_conversions
- pcl_ros
- tf2_ros
- tf2_eigen
- tf2_sensor_msgs
- tf2_geometry_msgs 