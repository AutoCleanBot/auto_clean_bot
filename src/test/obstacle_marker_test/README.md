# Obstacle Marker Test

这是一个测试节点，用于测试障碍物标记可视化代码在RViz中的显示效果。

## 功能特性

- 模拟多传感器（LiDAR、Camera、Radar）的障碍物检测数据
- 显示跟踪框（白色立方体）和检测框（彩色立方体）
- 显示传感器关联线（连接跟踪点和检测点）
- 显示存在概率文本信息
- 支持可配置的坐标系（frame_id）
- 支持可配置的发布频率

## 可视化元素

### 颜色编码
- **蓝色**: LiDAR检测 (LID)
- **绿色**: Camera检测 (CAM)  
- **黄色**: Radar检测 (RAD)
- **白色**: 有关联的跟踪框
- **灰色**: 无关联的跟踪框

### 标记类型
- **CUBE_LIST**: 跟踪框和检测框
- **LINE_LIST**: 传感器关联线
- **TEXT_VIEW_FACING**: 存在概率文本

## 编译和运行

### 编译
```bash
cd /path/to/your/workspace
colcon build --packages-select obstacle_marker_test
source install/setup.bash
```

### 运行（仅测试节点）
```bash
ros2 launch obstacle_marker_test obstacle_marker_test.launch.py
```

### 运行（包含RViz）
```bash
ros2 launch obstacle_marker_test obstacle_marker_test_with_rviz.launch.py
```

### 自定义参数运行
```bash
# 使用自定义坐标系和发布频率
ros2 launch obstacle_marker_test obstacle_marker_test_with_rviz.launch.py \
    frame_id:=map \
    publish_rate:=5.0
```

## 配置参数

### Launch参数
- `frame_id`: 标记的坐标系 (默认: "base_link")
- `publish_rate`: 发布频率，单位Hz (默认: 2.0)
- `use_rviz`: 是否启动RViz (默认: true)

### 配置文件参数
在 `config/obstacle_marker_test.yaml` 中可以修改：
- `frame_id`: 坐标系名称
- `publish_rate`: 发布频率
- `marker_lifetime`: 标记生命周期
- `text_scale`: 文本缩放
- `box_scale`: 立方体缩放
- `line_width`: 线条宽度

## 测试数据

节点会生成三组动态测试数据：

1. **对象组1**: 单个LiDAR检测，有关联 - **圆周运动**
   - 运动轨迹: 以(5.0, 0.0)为中心，半径3米的圆周运动
   - 角速度: 0.5 rad/s
   - 存在概率: 80%
   - 颜色: 蓝色（LiDAR）

2. **对象组2**: 多传感器融合对象 - **直线往返运动**
   - 运动轨迹: 从(-5, -2)到(5, 2)的直线往返运动
   - 周期: 10秒（5秒去，5秒回）
   - LiDAR + Camera检测
   - 存在概率: 85%
   - 颜色: 蓝色（LiDAR）+ 绿色（Camera）

3. **对象组3**: 无关联对象（灰色显示）- **振荡运动**
   - 运动轨迹: 在X=0处，Y轴方向上下振荡
   - 振荡范围: Y = 4.0 ± 2.0米
   - 频率: 0.8 rad/s
   - 仅Radar检测
   - 存在概率: 40%
   - 颜色: 灰色（无关联）

## RViz设置

预配置的RViz文件包含：
- Grid显示
- MarkerArray显示，订阅 `/obstacle_marker_test/debug_markers`
- 所有命名空间已启用
- 合适的视角设置

## 话题

- **发布**: `/obstacle_marker_test/debug_markers` (visualization_msgs/MarkerArray)

## 依赖

- rclcpp
- std_msgs
- geometry_msgs
- visualization_msgs
- unique_identifier_msgs
- boost (uuid相关功能)

## 故障排除

1. **看不到标记**: 检查RViz中的Fixed Frame是否与launch参数中的frame_id一致
2. **标记闪烁**: 可能是发布频率过高，尝试降低publish_rate
3. **编译错误**: 确保所有依赖包都已安装

## 扩展

可以通过修改以下文件来扩展功能：
- `src/obstacle_marker_test_node.cpp`: 修改测试数据生成逻辑
- `src/debug_object.cpp`: 修改可视化绘制逻辑
- `config/obstacle_marker_test.yaml`: 调整参数
- `config/obstacle_marker_test.rviz`: 调整RViz显示设置
