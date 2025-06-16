# 地图节点

这是一个简单的ROS2地图节点，用于读取地图文件并基于当前位置发布边界点信息。

## 功能

- 从CSV文件中读取左右边界点
- 基于当前位置，计算并发布前方一定长度的边界点
- 支持通过参数配置边界文件名、发布频率和边界点长度

## 依赖

- ROS2 Foxy
- `bot_msg` 包（包含自定义消息定义）

## 编译

```bash
cd ~/auto_clean_bot
colcon build --packages-select map
source install/setup.bash
```

## 使用方法

### 直接启动

```bash
ros2 launch map map.launch.py
```

可以通过命令行参数修改默认设置：

```bash
ros2 launch map map.launch.py map_files_dir:=/path/to/map_files left_boundary_file:=my_left_boundary.csv right_boundary_file:=my_right_boundary.csv boundary_length:=100.0 publish_frequency:=20.0
```

### 使用配置文件启动

```bash
ros2 launch map map_with_config.launch.py
```

使用不同的配置文件：

```bash
ros2 launch map map_with_config.launch.py config_file:=/path/to/my_config.yaml
```

或者使用预定义的配置：

```bash
ros2 launch map map_with_config.launch.py config_file:=$(ros2 pkg prefix map)/share/map/config/map_record_5.yaml
```

## 参数

| 参数名 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| map_files_dir | string | ~/auto_clean_bot/map_files | 地图文件目录 |
| left_boundary_file | string | local_record_4_left_boundary.csv | 左边界文件名 |
| right_boundary_file | string | local_record_4_right_boundary.csv | 右边界文件名 |
| left_boundary_name | string | left_boundary | 左边界名称 |
| right_boundary_name | string | right_boundary | 右边界名称 |
| boundary_length | double | 50.0 | 发布的边界段长度（米） |
| publish_frequency | double | 10.0 | 边界信息发布频率（Hz） |

## 话题

### 订阅

- `/localization/rtk_info` (bot_msg/msg/LocalizationInfo) - 定位信息

### 发布

- `/map/left_boundary` (bot_msg/msg/Boundary) - 左边界信息
- `/map/right_boundary` (bot_msg/msg/Boundary) - 右边界信息

## 文件格式

边界文件应为CSV格式，包含标题行和以下列：

```
east,north,yaw
x1,y1,heading1
x2,y2,heading2
...
```

其中：
- east: 东向坐标（米）
- north: 北向坐标（米）
- yaw: 航向角（度） 