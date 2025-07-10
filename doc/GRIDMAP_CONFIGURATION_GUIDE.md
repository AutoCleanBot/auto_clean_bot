# GridMap 输出配置指南

## 概述

现在 `costmap_generator` 支持通过配置文件灵活控制是否生成 GridMap 输出。您可以选择：

1. **只生成 OccupancyGrid**（默认，性能最佳）
2. **同时生成 GridMap 和 OccupancyGrid**（完整功能）

## 配置参数

### 新增配置参数

在 `costmap_generator.param.yaml` 中添加了新的配置参数：

```yaml
enable_gridmap_output: false  # 设置为true启用GridMap输出，false只输出OccupancyGrid
```

### 配置选项说明

| 参数值 | 输出内容 | 性能影响 | 适用场景 |
|--------|----------|----------|----------|
| `false` | 只输出 OccupancyGrid | 最佳性能 | 大多数应用场景 |
| `true` | 同时输出 GridMap + OccupancyGrid | 轻微性能影响 | 需要GridMap格式的应用 |

## 使用方法

### 1. 只使用 OccupancyGrid（推荐）

**配置文件设置：**
```yaml
/**:
  ros__parameters:
    enable_gridmap_output: false
    occupancy_grid_topic: "/occupancy_grid"
    # ... 其他参数
```

**输出话题：**
- `/occupancy_grid` (nav_msgs/OccupancyGrid)

**优势：**
- 最佳性能
- 减少内存使用
- 减少网络带宽
- 适合大多数导航和路径规划应用

### 2. 同时使用 GridMap 和 OccupancyGrid

**配置文件设置：**
```yaml
/**:
  ros__parameters:
    enable_gridmap_output: true
    costmap_topic: "/costmap"
    occupancy_grid_topic: "/occupancy_grid"
    # ... 其他参数
```

**输出话题：**
- `/costmap` (grid_map_msgs/GridMap)
- `/occupancy_grid` (nav_msgs/OccupancyGrid)

**适用场景：**
- 需要多层地图信息
- 使用 grid_map 库的应用
- 需要自定义地图处理

## 性能对比

### 只输出 OccupancyGrid (enable_gridmap_output: false)
```
[INFO] Point cloud transform lookup time: 1-5 ms
[INFO] Occupancy grid generation time: 1 ms
[INFO] Total processing time: 10-20 ms
```

### 同时输出 GridMap + OccupancyGrid (enable_gridmap_output: true)
```
[INFO] Point cloud transform lookup time: 1-5 ms
[INFO] GridMap publishing time: 1-3 ms
[INFO] Occupancy grid generation time: 1 ms
[INFO] Total processing time: 12-25 ms
```

**性能影响：** 启用 GridMap 输出会增加约 1-3ms 的处理时间。

## 启动日志

程序启动时会显示当前配置：

```bash
[INFO] [costmap_generator]: enable_gridmap_output: false
[INFO] [costmap_generator]: GridMap output disabled
```

或

```bash
[INFO] [costmap_generator]: enable_gridmap_output: true
[INFO] [costmap_generator]: GridMap output enabled
```

## 配置文件示例

### 完整配置文件示例

```yaml
/**:
  ros__parameters:
    # 基本参数
    update_rate: 40.0
    grid_resolution: 0.1
    grid_length_x: 30.0
    grid_length_y: 30.0
    grid_position_x: 0.0
    grid_position_y: 0.0
    
    # 高度过滤
    maximum_height_thres: 3.0
    minimum_height_thres: 0.3
    
    # 值范围
    grid_min_value: 0.0
    grid_max_value: 1.0
    
    # 坐标系
    input_frame: "base_link"
    map_frame: "map"
    costmap_frame: "map"
    
    # 输入输出话题
    input_points_topic: "/no_ground_points"
    costmap_topic: "/costmap"
    occupancy_grid_topic: "/occupancy_grid"
    
    # 功能开关
    is_pub_pnt_cloud: true
    enable_gridmap_output: false  # 关键配置：控制GridMap输出
```

## 迁移指南

### 从旧版本迁移

如果您之前使用的版本只输出 OccupancyGrid：
- **无需修改**：默认配置 `enable_gridmap_output: false` 保持原有行为

如果您之前使用的版本同时输出 GridMap：
- **添加配置**：在配置文件中设置 `enable_gridmap_output: true`

### 运行时切换

要在运行时切换输出模式，需要：
1. 修改配置文件
2. 重启 costmap_generator 节点

## 故障排除

### 常见问题

1. **GridMap 话题没有数据**
   - 检查 `enable_gridmap_output` 是否设置为 `true`
   - 查看启动日志确认 "GridMap output enabled"

2. **性能问题**
   - 如果不需要 GridMap，设置 `enable_gridmap_output: false`
   - 检查 TF 变换性能（主要瓶颈）

3. **配置不生效**
   - 确认配置文件路径正确
   - 重启节点使配置生效

### 调试命令

```bash
# 检查话题列表
ros2 topic list | grep -E "(costmap|occupancy)"

# 检查话题数据
ros2 topic echo /occupancy_grid --once
ros2 topic echo /costmap --once

# 检查参数
ros2 param get /costmap_generator enable_gridmap_output
```

## 总结

通过 `enable_gridmap_output` 参数，您可以根据实际需求灵活选择输出格式：

- **性能优先**：设置为 `false`，只输出 OccupancyGrid
- **功能完整**：设置为 `true`，同时输出 GridMap 和 OccupancyGrid

这种设计既保证了向后兼容性，又提供了性能优化的选择。
