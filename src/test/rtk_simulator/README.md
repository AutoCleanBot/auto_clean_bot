# RTK 定位模拟器

这是一个用于测试的 RTK 定位信息模拟器，可以从 CSV 文件中读取轨迹点，并以指定频率发布带有随机噪声的定位信息。

## 功能特性

- 从 CSV 文件读取轨迹数据（与 routing 节点相同的格式）
- 以 10Hz 频率发布 `bot_msg::msg::LocalizationInfo` 消息
- 同时发布 `geometry_msgs::msg::PoseStamped` 消息（与真实 RTK 节点一致）
- 为坐标和航向添加 1%-5% 的随机噪声
- 支持轨迹循环播放
- 发布到 `localization/rtk_info` 和 `/gnss/pose` 话题

## 编译

```bash
cd ~/auto_clean_bot
colcon build --packages-select rtk_simulator
source install/setup.bash
```

## 使用方法

### 1. 使用默认参数启动

```bash
ros2 launch rtk_simulator rtk_simulator_simple.launch.py
```

### 2. 使用配置文件启动

```bash
ros2 launch rtk_simulator rtk_simulator.launch.py
```

### 3. 自定义参数启动

```bash
ros2 launch rtk_simulator rtk_simulator.launch.py \
    csv_file_path:=~/auto_clean_bot/path/local_record_4.csv \
    publish_frequency:=20.0 \
    noise_min_percentage:=2.0 \
    noise_max_percentage:=8.0 \
    loop_trajectory:=false
```

### 4. 直接运行节点

```bash
ros2 run rtk_simulator rtk_simulator_node
```

## 参数说明

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `csv_file_path` | string | `~/auto_clean_bot/path/local_record_2.csv` | CSV轨迹文件路径 |
| `publish_frequency` | double | 10.0 | 发布频率 (Hz) |
| `noise_min_percentage` | double | 1.0 | 最小噪声百分比 |
| `noise_max_percentage` | double | 5.0 | 最大噪声百分比 |
| `loop_trajectory` | bool | true | 是否循环播放轨迹 |
| `gnss_topic_name` | string | `/gnss/pose` | GNSS pose 话题名称 |
| `gnss_frame_id` | string | `map` | GNSS pose 坐标系 |

## CSV 文件格式

CSV 文件应包含以下列（与 routing 节点兼容）：

```
longtitude,latitude,altitude,north,east,up,yaw,pitch,roll,vel_speed,vel_north,vel_east,vel_up,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,rtk_status
```

## 发布的话题

- `/localization/rtk_info` (`bot_msg::msg::LocalizationInfo`) - 模拟的RTK定位信息
- `/gnss/pose` (`geometry_msgs::msg::PoseStamped`) - GNSS位姿信息（与定位信息数值一致）

## 噪声模型

模拟器为以下字段添加随机噪声：
- 坐标信息：`longtitude`, `latitude`, `north`, `east`
- 航向信息：`yaw`
- 速度信息：`vel_speed`, `vel_north`, `vel_east`（噪声幅度减半）

噪声范围为指定百分比的随机偏移，例如 5% 的噪声意味着值会在原值的 95%-105% 范围内随机变化。

## 使用场景

1. **规划节点测试**：为 planning 节点提供模拟的定位输入
2. **控制算法验证**：测试控制算法对定位噪声的鲁棒性
3. **系统集成测试**：在没有真实 RTK 设备时进行系统测试
4. **算法开发**：为算法开发提供可重复的测试数据

## 监控和调试

查看发布的消息：
```bash
ros2 topic echo /localization/rtk_info
```

查看发布频率：
```bash
ros2 topic hz /localization/rtk_info
```

查看话题信息：
```bash
ros2 topic info /localization/rtk_info
```

## 注意事项

1. 确保 CSV 文件路径正确且文件存在
2. CSV 文件格式必须与 routing 节点兼容
3. 噪声百分比设置要合理，过大的噪声可能导致系统不稳定
4. 循环播放模式下，轨迹会无限重复，适合长时间测试
