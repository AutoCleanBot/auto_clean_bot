# 路径处理模块

本模块提供了完整的轨迹处理和车道边界生成功能，适用于自动驾驶路径规划和地图构建。

## 模块概述

### 核心功能
1. **轨迹处理** (`process_trajectory.py`) - 对原始GPS轨迹进行密化、平滑和质量提升
2. **边界生成** (`generate_boundaries.py`) - 基于中心线生成精确的车道边界线
3. **可视化工具** - 自动生成处理结果的可视化图表

### 技术特点
- 使用Savitzky-Golay滤波器进行高质量平滑处理
- 基于几何曲率的自适应边界计算
- 支持东北天坐标系（ENU）
- 自动处理航向角连续性问题
- 提供完整的数据备份和恢复机制

## 详细使用说明

### 1. process_trajectory.py - 轨迹处理工具

#### 算法原理
- **密化算法**: 使用三次样条插值在轨迹点间插入新点，确保点间距离均匀
- **平滑算法**: 采用Savitzky-Golay滤波器，保持轨迹几何特征的同时去除噪声
- **曲率计算**: 基于梯度计算的二阶导数方法，提供准确的路径曲率信息

#### 参数配置
```python
DENSIFY_MAX_DISTANCE = 0.1    # 密化后点间最大距离(米)
SAVGOL_WINDOW = 101           # 平滑窗口大小
SAVGOL_POLYORDER = 3          # 多项式阶数
```

#### 输入数据格式
```csv
east,north,yaw
123.456,789.012,45.0
123.466,789.022,46.2
...
```

#### 处理流程
1. 数据验证和加载
2. 路径密化（插值）
3. 坐标平滑处理
4. 航向角和曲率重新计算
5. 结果保存和可视化

### 2. generate_boundaries.py - 边界生成工具

#### 算法原理
- **几何航向计算**: 基于坐标梯度计算真实的几何航向角
- **曲率自适应**: 在高曲率区域自动调整边界宽度，避免边界线交叉
- **三点法曲率**: 使用三角形面积法计算更稳定的曲率值

#### 边界计算公式
```python
# 法向量计算
norm_dx = -sin(yaw_rad)
norm_dy = cos(yaw_rad)

# 曲率自适应宽度
curve_factor = 1.0 / (1.0 + curvature * width)
local_width = width * curve_factor

# 边界点计算
left_point = center_point + norm_vector * local_width
right_point = center_point - norm_vector * local_width
```

#### 输出数据格式
```csv
east,north,yaw
123.446,789.012,45.0  # 左边界点
123.466,789.012,45.0  # 右边界点
...
```

## 高级用法

### 批量处理脚本示例

```bash
#!/bin/bash
# 批量处理多个轨迹文件

for file in *.csv; do
    echo "Processing $file..."

    # 步骤1: 轨迹处理
    python process_trajectory.py "$file"

    # 步骤2: 生成边界（车道宽度3.5米）
    python generate_boundaries.py "$file" --width 1.75

    echo "Completed $file"
done
```

### Python API 使用

```python
from generate_boundaries import generate_boundaries
from process_trajectory import densify_path, smooth_and_recalculate

# 直接调用函数
left_df, right_df, center_df = generate_boundaries('trajectory.csv', lane_width=1.5)

# 自定义处理流程
import pandas as pd
df = pd.read_csv('raw_trajectory.csv')
dense_df = densify_path(df, max_distance=0.05)  # 更密集的点
smooth_df = smooth_and_recalculate(dense_df, savgol_window=51)  # 更强的平滑
```

## 质量控制

### 输入数据要求
- **最小点数**: 至少3个点（边界生成），推荐50+个点
- **坐标精度**: 建议厘米级精度
- **采样频率**: 推荐1-10Hz
- **轨迹连续性**: 避免大的跳跃或断点

### 输出质量指标
- **点间距离**: 处理后约0.1米均匀分布
- **平滑度**: 曲率变化连续，无突变
- **边界一致性**: 左右边界与中心线保持平行关系

### 常见问题排查

1. **处理失败**
   ```
   Error: Not enough data points
   ```
   - 解决: 确保输入文件至少包含3个有效数据点

2. **边界异常**
   ```
   Warning: Boundary lines may intersect
   ```
   - 解决: 减小车道宽度参数或增加轨迹平滑程度

3. **内存不足**
   ```
   MemoryError during processing
   ```
   - 解决: 分段处理大型轨迹文件

## 性能优化

### 处理速度优化
- 对于大型文件（>10000点），建议分段处理
- 使用SSD存储可显著提升I/O性能
- 多核并行处理可用于批量文件

### 内存使用优化
- 大文件处理时使用chunked读取
- 及时释放中间变量
- 考虑使用内存映射文件

## 扩展开发

### 添加新的平滑算法
```python
def custom_smooth_function(data, **params):
    # 实现自定义平滑算法
    return smoothed_data

# 在smooth_and_recalculate中集成
```

### 自定义边界计算
```python
def custom_boundary_calculation(east, north, yaw, width):
    # 实现特殊的边界计算逻辑
    return left_east, left_north, right_east, right_north
```

## 测试数据

模块包含以下测试数据文件：
- `local_record_1.csv` - 直线路段测试
- `local_record_2.csv` - 弯道路段测试
- `local_record_3.csv` - 复杂路况测试
- `local_record_4.csv` - 高曲率测试
- `local_record_5.csv` - 长距离测试

每个测试文件都包含对应的处理结果和可视化图片。