# PointCloud Preprocess

这个ROS 2包提供了激光点云的预处理功能，主要用于坐标转换。

## 功能

- 将点云从一个坐标系转换到另一个坐标系
- 支持动态配置输入和输出话题
- 支持使用传感器原始坐标系或指定的输入坐标系
- 支持使用点云时间戳对应的变换或最新的变换

## 参数

- `update_rate`: 参数更新频率 (Hz)
- `input_frame`: 输入点云坐标系，空字符串表示使用sensor_frame
- `output_frame`: 输出点云坐标系
- `use_sensor_frame`: 是否使用传感器坐标系作为输入坐标系
- `timeout`: TF查询超时时间 (秒)
- `use_latest_transforms`: 是否使用最新的变换而不是点云时间戳对应的变换
- `input_topic`: 输入点云话题
- `output_topic`: 输出点云话题

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
ros2 param set /pointcloud_transformer input_topic /new_input_topic
ros2 param set /pointcloud_transformer output_topic /new_output_topic
ros2 param set /pointcloud_transformer output_frame map
```

## 依赖项

- rclcpp
- sensor_msgs
- pcl_conversions
- pcl_ros
- tf2_ros
- tf2_eigen
- tf2_sensor_msgs
- tf2_geometry_msgs 