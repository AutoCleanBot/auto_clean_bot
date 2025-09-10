# HC GNSS 驱动节点

## 节点说明
负责解析来自 HC GNSS 设备的 `$GPCHC` 格式传感器消息。

## 编译方法

- 依赖 ros2 tf2 和 ros2 tf-geometry-msgs
```bash
sudo apt install ros-foxy-tf2 ros-foxy-tf2-geometry-msgs
```

## 接口说明

使用 HC GNSS 设备的 RS232 接口连接。

## GPCHC 数据格式说明

数据格式：`$GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,gyro x,gyro y,gyroz,acc x,accy,accz,Latitude,Longitude,Altitude,Ve,Vn,Vu,V,NS1,NS2,Status,Age,Warming,Cs<CR><LF>`

字段说明：
- GPSWeek: GPS周
- GPSTime: GPS时间（秒）
- Heading: 航向角（度）
- Pitch: 俯仰角（度）
- Roll: 横滚角（度）
- gyro x/y/z: 角速度（度/秒）
- acc x/y/z: 加速度（米/秒²）
- Latitude: 纬度（度）
- Longitude: 经度（度）
- Altitude: 高度（米）
- Ve/Vn/Vu: 东向/北向/上向速度（米/秒）
- V: 速度模长（米/秒）
- NS1/NS2: 卫星数
- Status: 状态
- Age: 差分龄期
- Warming: 警告
- Cs: 校验和

## 配置参数说明

```yaml
hc_gnss_node:
  ros__parameters:
    # 设备配置
    device_name: "/dev/ttyTHS4"        # 串口设备名
    baud_rate: 460800                  # 波特率
    timeout_ms: 20                     # 超时时间（毫秒）
    
    # 基准点配置
    base_latitude: 0.0                 # 基点纬度（度）
    base_longtitude: 0.0               # 基点经度（度）
    base_altitude: 0.0                 # 基点高度（米）
    heading_offset: 0.0                # 航向偏移角（度）
    
    # 话题配置
    local_frame_id: "hc_gnss_frame"
    local_topic_name: "hc_gnss_fix"
    local_publish_rate: 10.0           # 定位信息发布频率（Hz）
    
    imu_frame_id: "hc_imu_frame"
    imu_topic_name: "hc_imu"
    imu_publish_rate: 10.0             # IMU数据发布频率（Hz）
    
    gnss_frame_id: "hc_gnss_pose_enu_frame"
    gnss_topic_name: "hc_gnss_pose_enu"
    gnss_publish_rate: 10.0            # ENU坐标发布频率（Hz）
    
    # 日志配置
    enable_debug_log: false            # 是否启用调试日志
    log_interval: 10                   # 日志输出间隔（每N次解析输出一次）
    
    # 原始数据保存配置
    enable_info_str_save: false        # 是否保存原始数据到文件
    info_str_save_dir: "/tmp/hc_gnss_logs"  # 保存目录
```

## 发布话题

- **定位信息**: `hc_gnss_fix` (bot_msg/LocalizationInfo)
- **IMU数据**: `hc_imu` (sensor_msgs/Imu)  
- **ENU坐标**: `hc_gnss_pose_enu` (geometry_msgs/PoseStamped)

## 使用方法

### 1. 编译包
```bash
cd /path/to/workspace
colcon build --packages-select hc_gnss --symlink-install
```

### 2. 启动节点
```bash
# 使用默认配置
ros2 run hc_gnss hc_gnss_node

# 使用配置文件
ros2 run hc_gnss hc_gnss_node --ros-args --params-file src/drivers/hc_gnss/config/hc_gnss.param.yaml

# 使用launch文件
ros2 launch hc_gnss hc_gnss.launch.py
```

### 3. 查看话题数据
```bash
# 查看定位信息
ros2 topic echo /hc_gnss_fix

# 查看IMU数据
ros2 topic echo /hc_imu

# 查看ENU坐标
ros2 topic echo /hc_gnss_pose_enu
```

## info_str 文件保存功能说明

为了满足调试需求，新增了将原始 `$GPCHC` 数据保存到文件的功能：

### 功能特性
- **时间编码文件名**: 自动生成基于时间戳的文件名，格式为 `hc_gnss_info_YYYYMMDD_HHMMSS.log`
- **可配置保存目录**: 通过 `info_str_save_dir` 参数指定保存路径
- **完整数据记录**: 记录程序运行期间的所有 `$GPCHC` 数据
- **原始数据保存**: 直接保存原始的数据，不添加额外的时间戳
- **自动文件管理**: 程序启动时创建文件，程序结束时自动关闭文件

### 配置参数
- `enable_info_str_save`: 是否启用文件保存功能 (默认: false)
- `info_str_save_dir`: 文件保存目录 (默认: "/tmp/hc_gnss_logs")

### 日志文件格式示例
```
$GPCHC,2345,123456.789,45.67,1.23,-0.45,0.1,0.2,0.3,9.81,-0.1,0.2,31.960000,117.380000,50.5,1.2,1.5,0.1,1.8,12,8,4,0.0,0,75
$GPCHC,2345,123456.889,45.68,1.24,-0.46,0.1,0.2,0.3,9.82,-0.1,0.2,31.960001,117.380001,50.6,1.2,1.5,0.1,1.8,12,8,4,0.0,0,76
```

## 注意事项

1. 确保串口设备权限正确：`sudo chmod 666 /dev/ttyTHS4`
2. 根据实际硬件配置修改设备名称和波特率
3. 根据实际安装位置调整 `heading_offset` 参数
4. 设置正确的基准点坐标以获得准确的ENU坐标转换
5. 调整发布频率以匹配系统性能要求