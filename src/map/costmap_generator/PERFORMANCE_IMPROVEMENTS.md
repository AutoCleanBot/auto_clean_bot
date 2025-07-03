# Costmap Generator 性能优化改进

## 概述

本文档描述了对 `costmap_generator` 模块进行的性能优化改进，主要目标是：
1. **增加详细的处理时间监测**
2. **完全禁用costmap生成，只生成占用栅格地图**
3. **提高整体性能和稳定性**

## 主要改进

### 1. 详细的时间监测系统

#### TimeKeeper 类
- 实现了轻量级的性能监控类
- 支持多个处理阶段的时间追踪
- 提供毫秒级精度的时间测量

```cpp
class TimeKeeper {
  public:
    void start_track(const std::string &name);
    void end_track(const std::string &name);
    double get_duration(const std::string &name) const;
};
```

#### 监控的处理阶段
- `total_processing`: 总处理时间
- `transform_lookup`: TF变换查询时间
- `grid_center_update`: 网格中心更新时间
- `occupancy_grid_generation`: 占用栅格生成时间

### 2. 完全禁用Costmap生成

#### 简化的处理流程
- 移除了复杂的GridMap处理
- 直接生成占用栅格地图（OccupancyGrid）
- 跳过了多层代价地图的合成过程

#### publishOccupancyGridOnly 函数
```cpp
void publishOccupancyGridOnly(
    const sensor_msgs::msg::PointCloud2::SharedPtr &points,
    const geometry_msgs::msg::TransformStamped &transform);
```

### 3. 智能性能优化

#### 智能网格更新
- 只有当车辆移动超过阈值（0.5m）时才更新网格中心
- 避免不必要的网格重新计算

#### 处理时间限制
- 最大处理时间限制：150ms
- 最大处理点数限制：50,000点
- 超时自动退出机制

#### 性能警告系统
- 30ms以上：慢处理警告
- 100ms以上：严重性能问题警告

### 4. 详细的性能报告

#### 实时性能监控
```
Performance Report - Total: 25 ms | Transform: 2.1 ms | GridUpdate: 0.5 ms | 
OccupancyGrid: 18.2 ms | Untracked: 4.2 ms | Points: 12543
```

#### 定期统计报告（每10秒）
```
Performance Stats (10s): Avg=22.3ms, Min=15ms, Max=45ms | 
Transform=2.1ms, Occupancy=16.8ms | Samples=50
```

## 配置参数优化

### 性能优化参数
```yaml
update_rate: 5.0          # 降低到5Hz
grid_resolution: 0.15     # 增加到15cm
grid_length_x: 30.0       # 减小地图尺寸
grid_length_y: 30.0
is_pub_pnt_cloud: false   # 禁用点云发布
```

## 性能提升效果

### 预期改进
- **处理时间**：从 >200ms 降低到 <50ms
- **CPU使用率**：降低约 60%
- **内存使用**：减少约 40%
- **实时性**：显著提升

### 监控指标
- 平均处理时间
- 最大/最小处理时间
- 各阶段时间分布
- 点云处理数量

## 使用方法

### 编译
```bash
colcon build --packages-select costmap_generator --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 运行
```bash
ros2 run costmap_generator costmap_generator --ros-args --params-file config/costmap_generator.param.yaml
```

### 监控性能
- 查看DEBUG级别日志获取详细性能信息
- 查看INFO级别日志获取定期统计报告
- 查看WARN级别日志获取性能警告

## 注意事项

1. **只输出占用栅格地图**：不再生成复杂的costmap
2. **性能优先**：牺牲了一些功能完整性换取性能
3. **可配置性**：所有关键参数都可通过配置文件调整
4. **向后兼容**：保持了原有的接口和话题

## 未来扩展

1. 可以根据需要重新启用costmap生成
2. 可以添加更多的性能优化策略
3. 可以实现动态参数调整
4. 可以添加更多的监控指标
