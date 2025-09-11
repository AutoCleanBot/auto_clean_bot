# 数据记录节点 (Data Recorder Node)

## 节点功能

该节点用于实时记录车辆数据，包括：
- CAN节点的方向盘转角反馈
- RTK节点的当前位置（经纬度和东北天坐标）
- RTK节点的航向角度
- 车辆速度、档位等底盘信息
- IMU数据（加速度、角速度）

所有数据以CSV格式保存，便于后续分析。

## 订阅话题

- `/chassis/chassis_info` (bot_msg/ChassisInfo): 底盘信息，包含方向盘转角、速度、档位等
- `/localization/rtk_info` (bot_msg/LocalizationInfo): RTK定位信息，包含位置、姿态、速度等

## 输出文件

CSV文件包含以下字段：
- `timestamp`: 时间戳 (毫秒)
- `steer_angle_deg`: 方向盘转角 (度)
- `cur_speed_mps`: 当前速度 (m/s)
- `gear`: 档位 (0=N, 1=F, 2=R)
- `direction`: 方向 (1=前进, 2=后退)
- `longitude_deg`: 经度 (度)
- `latitude_deg`: 纬度 (度)
- `altitude_m`: 高度 (米)
- `north_m`: 北向位置 (米)
- `east_m`: 东向位置 (米)
- `up_m`: 上向位置 (米)
- `yaw_deg`: 航向角 (度)
- `pitch_deg`: 俯仰角 (度)
- `roll_deg`: 横滚角 (度)
- `vel_speed_mps`: 水平速度 (m/s)
- `vel_north_mps`: 北向速度 (m/s)
- `vel_east_mps`: 东向速度 (m/s)
- `vel_up_mps`: 上向速度 (m/s)
- `acc_x_mps2`: X轴加速度 (m/s²)
- `acc_y_mps2`: Y轴加速度 (m/s²)
- `acc_z_mps2`: Z轴加速度 (m/s²)
- `gyro_x_dps`: X轴角速度 (度/秒)
- `gyro_y_dps`: Y轴角速度 (度/秒)
- `gyro_z_dps`: Z轴角速度 (度/秒)
- `rtk_status`: RTK状态 (0-4)

## 配置参数

在 `config/data_recorder.yaml` 中可配置：
- `chassis_info_topic`: 底盘信息话题名称
- `localization_info_topic`: 定位信息话题名称
- `record_rate`: 记录频率 (Hz)
- `enable_record`: 是否启用记录
- `save_directory`: 保存目录

## 编译方法

```bash
# 进入工作空间根目录
cd /home/nvidia/auto_clean_bot

# 编译数据记录节点
colcon build --packages-select data_recorder

# 设置环境变量
source install/setup.bash
```

## 使用方法

### 1. 使用launch文件启动（推荐）

```bash
ros2 launch data_recorder data_recorder.launch.py
```

### 2. 直接启动节点

```bash
ros2 run data_recorder data_recorder_node
```

### 3. 使用自定义参数启动

```bash
ros2 run data_recorder data_recorder_node --ros-args \
  --params-file src/utils/data_recorder/config/data_recorder.yaml
```

## 输出文件位置

默认保存在 `/tmp/data_recorder_logs/` 目录下，文件名格式为：
`vehicle_data_YYYYMMDD_HHMMSS.csv`

例如：`vehicle_data_20241211_143052.csv`

## 注意事项

1. 确保CAN节点和RTK节点正常运行并发布数据
2. 记录频率建议设置为10Hz，避免产生过大的文件
3. 定期清理日志文件，避免磁盘空间不足
4. 如需更改话题名称，请修改配置文件中的参数

## 故障排除

1. **无数据记录**: 检查话题名称是否正确，确认CAN和RTK节点是否正常发布数据
2. **文件无法创建**: 检查保存目录权限，确保有写入权限
3. **记录频率异常**: 检查系统负载，适当降低记录频率

## 依赖

- ROS2 Foxy
- bot_msg package（包含ChassisInfo和LocalizationInfo消息定义）