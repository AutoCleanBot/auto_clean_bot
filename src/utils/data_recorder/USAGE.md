# 数据记录节点使用说明

## 节点概述

数据记录节点 (`data_recorder`) 是一个专门用于记录车辆实时数据的ROS2节点，能够同步记录：

1. **CAN节点数据**：
   - 方向盘转角反馈 (steer_angle)
   - 当前车速 (cur_speed)
   - 档位状态 (gear)
   - 行驶方向 (direction)

2. **RTK定位数据**：
   - 经纬度坐标 (longitude, latitude)
   - 高度信息 (altitude)
   - 东北天坐标 (east, north, up)
   - 航向角度 (yaw, pitch, roll)
   - 速度信息 (vel_speed, vel_north, vel_east, vel_up)
   - IMU数据 (acc_x/y/z, gyro_x/y/z)
   - RTK状态 (rtk_status)

## 快速开始

### 1. 编译节点

```bash
# 方法一：使用构建脚本
./scripts/build/data_recorder.sh

# 方法二：手动编译
cd /home/nvidia/auto_clean_bot
colcon build --packages-select data_recorder
source install/setup.bash
```

### 2. 启动节点

```bash
# 方法一：使用运行脚本（推荐）
./scripts/run/data_recorder.sh

# 方法二：使用launch文件
source install/setup.bash
ros2 launch data_recorder data_recorder.launch.py

# 方法三：直接运行节点
source install/setup.bash
ros2 run data_recorder data_recorder_node
```

### 3. 检查运行状态

```bash
# 检查节点是否运行
ros2 node list | grep data_recorder

# 查看节点话题订阅
ros2 node info /data_recorder_node

# 查看日志输出
ros2 topic echo /rosout | grep data_recorder
```

## 配置说明

### 默认配置

配置文件位置：`src/utils/data_recorder/config/data_recorder.yaml`

```yaml
data_recorder_node:
  ros__parameters:
    chassis_info_topic: "/chassis/chassis_info"      # 底盘信息话题
    localization_info_topic: "/localization/rtk_info" # 定位信息话题
    record_rate: 10.0                                # 记录频率 (Hz)
    enable_record: true                              # 启用记录
    save_directory: "/tmp/data_recorder_logs"        # 保存目录
```

### 自定义配置

您可以根据实际情况修改配置：

1. **修改话题名称**：根据实际系统中的话题名称调整
2. **调整记录频率**：建议5-20Hz，避免文件过大
3. **更改保存目录**：确保目录有写入权限

## 输出文件格式

### 文件命名

文件名格式：`vehicle_data_YYYYMMDD_HHMMSS.csv`

示例：`vehicle_data_20241211_143052.csv`

### CSV字段说明

| 字段名 | 单位 | 说明 |
|--------|------|------|
| timestamp | ms | 时间戳（毫秒） |
| steer_angle_deg | ° | 方向盘转角 |
| cur_speed_mps | m/s | 当前速度 |
| gear | - | 档位 (0=空档, 1=前进, 2=后退) |
| direction | - | 方向 (1=前进, 2=后退, 0=空档) |
| longitude_deg | ° | 经度（8位小数精度） |
| latitude_deg | ° | 纬度（8位小数精度） |
| altitude_m | m | 高度 |
| north_m | m | 北向位置（ENU坐标） |
| east_m | m | 东向位置（ENU坐标） |
| up_m | m | 上向位置（ENU坐标） |
| yaw_deg | ° | 航向角 |
| pitch_deg | ° | 俯仰角 |
| roll_deg | ° | 横滚角 |
| vel_speed_mps | m/s | 水平速度 |
| vel_north_mps | m/s | 北向速度 |
| vel_east_mps | m/s | 东向速度 |
| vel_up_mps | m/s | 上向速度 |
| acc_x_mps2 | m/s² | X轴加速度 |
| acc_y_mps2 | m/s² | Y轴加速度 |
| acc_z_mps2 | m/s² | Z轴加速度 |
| gyro_x_dps | °/s | X轴角速度 |
| gyro_y_dps | °/s | Y轴角速度 |
| gyro_z_dps | °/s | Z轴角速度 |
| rtk_status | - | RTK状态 (0-4) |

### RTK状态说明

- 0: 初始化/未定位
- 1: 单点定位
- 2: RTD（实时差分）
- 3: RTK固定解
- 4: RTK浮点解

## 数据分析示例

### Python读取示例

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
df = pd.read_csv('vehicle_data_20241211_143052.csv')

# 绘制轨迹图
plt.figure(figsize=(10, 8))
plt.plot(df['east_m'], df['north_m'], 'b-', linewidth=2)
plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.title('Vehicle Trajectory (ENU Coordinates)')
plt.grid(True)
plt.axis('equal')
plt.show()

# 绘制方向盘转角与航向角关系
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(df['timestamp'], df['steer_angle_deg'], 'r-', label='Steering Angle')
plt.ylabel('Steering Angle (°)')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(df['timestamp'], df['yaw_deg'], 'b-', label='Yaw Angle')
plt.xlabel('Timestamp (ms)')
plt.ylabel('Yaw Angle (°)')
plt.legend()
plt.grid(True)
plt.show()
```

## 故障排除

### 常见问题

1. **节点启动失败**
   ```bash
   # 检查依赖是否安装
   ros2 pkg list | grep bot_msg
   
   # 重新编译
   colcon build --packages-select data_recorder
   ```

2. **无数据记录**
   ```bash
   # 检查话题是否存在
   ros2 topic list | grep -E "(chassis|localization)"
   
   # 检查话题数据
   ros2 topic echo /chassis/chassis_info --once
   ros2 topic echo /localization/rtk_info --once
   ```

3. **文件权限错误**
   ```bash
   # 创建目录并设置权限
   sudo mkdir -p /tmp/data_recorder_logs
   sudo chown $USER:$USER /tmp/data_recorder_logs
   chmod 755 /tmp/data_recorder_logs
   ```

4. **记录频率异常**
   - 检查系统负载
   - 降低记录频率（建议10Hz以下）
   - 确保磁盘空间充足

### 调试命令

```bash
# 查看节点日志
ros2 run data_recorder data_recorder_node --ros-args --log-level debug

# 监控话题频率
ros2 topic hz /chassis/chassis_info
ros2 topic hz /localization/rtk_info

# 检查节点参数
ros2 param list /data_recorder_node
ros2 param get /data_recorder_node record_rate
```

## 系统集成

### 与其他节点的配合

1. **启动顺序建议**：
   ```bash
   # 1. 启动CAN节点
   ros2 launch canbus_cq canbus.launch.py
   
   # 2. 启动RTK节点
   ros2 launch rtk rtk.launch.py
   
   # 3. 启动数据记录节点
   ros2 launch data_recorder data_recorder.launch.py
   ```

2. **系统启动脚本示例**：
   ```bash
   #!/bin/bash
   cd /home/nvidia/auto_clean_bot
   source install/setup.bash
   
   # 后台启动各节点
   ros2 launch canbus_cq canbus.launch.py &
   ros2 launch rtk rtk.launch.py &
   sleep 5  # 等待节点启动
   ros2 launch data_recorder data_recorder.launch.py
   ```

### 性能优化

1. **记录频率调优**：
   - 高精度场景：20Hz
   - 一般场景：10Hz
   - 低存储需求：5Hz

2. **存储优化**：
   - 定期清理旧日志文件
   - 使用SSD存储提高写入性能
   - 考虑数据压缩（如gzip）

3. **系统监控**：
   - 监控磁盘使用率
   - 监控CPU负载
   - 监控内存使用

## 维护建议

1. **定期检查**：
   - 每周检查日志文件大小
   - 验证数据记录的完整性
   - 检查系统资源使用情况

2. **数据备份**：
   - 重要数据及时备份
   - 考虑自动备份脚本
   - 异地存储重要数据

3. **版本更新**：
   - 关注ROS2和依赖包更新
   - 定期测试节点兼容性
   - 维护配置文件版本