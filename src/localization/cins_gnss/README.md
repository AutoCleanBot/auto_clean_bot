# CINS GNSS驱动包

## 包说明
本包为城芯智联PBOX串口数据格式解析驱动，基于RTK包的架构设计，用于解析城芯智联PBOX设备的二进制数据协议。

## 编译方法

```bash
# 在工作空间根目录下执行
colcon build --packages-select cins_gnss
```

## 接口说明

使用城芯智联PBOX设备的串口接口连接。

## 配置参数说明

主要参数包括：
- `device_name`: 串口设备名，默认"/dev/ttyTHS4"
- `baud_rate`: 波特率，默认460800
- `base_latitude`, `base_longtitude`, `base_altitude`: 基准点坐标
- `local_topic_name`: 定位信息话题名，默认"cins_fix"（与RTK包话题名不冲突）
- `imu_topic_name`: IMU话题名，默认"imu"
- `gnss_topic_name`: GNSS姿态话题名，默认"gnss_pose_enu"
- `heading_offset`: 航向偏移角度

## 数据协议

支持城芯智联PBOX二进制数据协议，包含以下主要字段：
- 数据头：0xAA55
- UTC时间戳
- INS状态信息
- IMU数据（陀螺仪、加速度计）
- 位置信息（纬度、经度、高度）
- 姿态信息（航向、俯仰、横滚）
- 异或校验

## 发布话题

- `cins_fix` (bot_msg/LocalizationInfo): 定位信息
- `imu` (sensor_msgs/Imu): IMU数据
- `gnss_pose_enu` (geometry_msgs/PoseStamped): ENU坐标系下的GNSS姿态

## 使用方法

```bash
# 使用默认参数启动
ros2 launch cins_gnss cins_gnss.launch.py

# 使用配置文件启动
ros2 run cins_gnss cins_gnss_node --ros-args --params-file src/drivers/cins_gnss/config/cins_gnss.param.yaml
```

## 注意事项

1. 确保串口设备权限正确
2. 根据实际安装位置调整`heading_offset`参数
3. 设置正确的基准点坐标以获得准确的ENU坐标
4. 支持原始数据保存功能，便于调试和分析