# 雷达坐标系更新说明

## 概述

根据新的雷达坐标系定义，更新了 `filterVehiclePoints` 函数中的车辆点云过滤算法。

## 坐标系变更

### 原坐标系定义
- **X轴正方向**：车辆右侧
- **Y轴正方向**：车头方向  
- **Z轴正方向**：垂直向上

### 新坐标系定义（雷达坐标系）
- **X轴正方向**：车头方向
- **Y轴正方向**：车辆左侧
- **Z轴正方向**：垂直向上

## 修改内容

### 1. 算法逻辑修改

#### 原逻辑（旧坐标系）
```cpp
// X轴对应车辆左右方向，负值是左侧
min_point[0] = -vehicle_left_width_ + vehicle_y_offset_ - vehicle_width_margin_;
// Y轴对应车辆前后方向，负值是后方
min_point[1] = -vehicle_back_length_ + vehicle_x_offset_ - vehicle_length_margin_;

// X轴正方向是车辆右侧
max_point[0] = vehicle_right_width_ + vehicle_y_offset_ + vehicle_width_margin_;
// Y轴正方向是车头方向
max_point[1] = vehicle_front_length_ + vehicle_x_offset_ + vehicle_length_margin_;
```

#### 新逻辑（雷达坐标系）
```cpp
// X轴对应车辆前后方向，负值是后方
min_point[0] = -vehicle_back_length_ + vehicle_x_offset_ - vehicle_length_margin_;
// Y轴对应车辆左右方向，负值是右侧
min_point[1] = -vehicle_right_width_ + vehicle_y_offset_ - vehicle_width_margin_;

// X轴正方向是车头方向
max_point[0] = vehicle_front_length_ + vehicle_x_offset_ + vehicle_length_margin_;
// Y轴正方向是车辆左侧
max_point[1] = vehicle_left_width_ + vehicle_y_offset_ + vehicle_width_margin_;
```

### 2. 配置文件注释更新

#### 参数含义变更
- `vehicle_front_length`: 从 "Y轴正方向" 改为 "X轴正方向"
- `vehicle_back_length`: 从 "Y轴负方向" 改为 "X轴负方向"
- `vehicle_right_width`: 从 "X轴正方向" 改为 "Y轴负方向"
- `vehicle_left_width`: 从 "X轴负方向" 改为 "Y轴正方向"

#### 偏移参数含义变更
- `vehicle_x_offset`: 从 "向右移动" 改为 "向前移动"
- `vehicle_y_offset`: 从 "向前移动" 改为 "向左移动"
- `vehicle_length_margin`: 从 "Y轴方向" 改为 "X轴方向"
- `vehicle_width_margin`: 从 "X轴方向" 改为 "Y轴方向"

### 3. 日志输出更新

#### 坐标系说明
- 从 "Vehicle coordinate system: Y+ = front, X+ = right, Z+ = up"
- 改为 "Radar coordinate system: X+ = front, Y+ = left, Z+ = up"

#### 尺寸显示顺序
- X轴：显示前后尺寸 (front/back)
- Y轴：显示左右尺寸 (left/right)
- Z轴：显示上下尺寸 (top/bottom)

### 4. 调试信息更新
- 过滤框调试信息添加坐标系说明：
  `"Vehicle filter box (X+=front, Y+=left, Z+=up)"`

## 影响范围

### 直接影响
1. **车辆点云过滤算法**：过滤区域的计算逻辑
2. **配置参数含义**：参数注释和实际作用
3. **日志输出**：坐标系说明和尺寸显示

### 无影响
1. **参数名称**：所有参数名称保持不变
2. **参数数值**：现有配置数值仍然有效
3. **其他功能**：坐标转换、降采样等功能不受影响

## 验证建议

### 1. 功能验证
- 检查车辆前方点云是否被正确过滤
- 检查车辆左侧点云是否被正确过滤
- 验证过滤区域的形状和位置

### 2. 参数验证
- 调整 `vehicle_front_length` 参数，观察前方过滤效果
- 调整 `vehicle_left_width` 参数，观察左侧过滤效果
- 验证偏移参数的作用方向

### 3. 日志验证
- 检查启动日志中的坐标系说明
- 验证调试日志中的过滤框信息

## 迁移指南

### 对于现有配置
1. **无需修改参数值**：现有的参数数值仍然适用
2. **理解新含义**：需要重新理解参数的物理含义
3. **验证效果**：建议重新验证过滤效果是否符合预期

### 对于新配置
1. **按新坐标系理解**：X+=前，Y+=左，Z+=上
2. **参考新注释**：使用配置文件中的新注释说明
3. **测试验证**：配置后进行实际测试验证

## 总结

成功将车辆点云过滤算法从原坐标系迁移到雷达坐标系，主要变更：
- X轴和Y轴的物理含义互换
- 算法逻辑相应调整
- 配置注释和日志输出更新
- 保持向后兼容性，现有配置仍然有效

新的坐标系定义更符合雷达设备的标准约定，有助于提高系统的一致性和可理解性。
