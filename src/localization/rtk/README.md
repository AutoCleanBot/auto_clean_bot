# 定位 - RTK节点
## 节点说明
负责解析来自组合惯导设备的传感消息.
## 编译方法

- 依赖ros2 tf2 和 ros2 tf-geomotry-msgs
```
sudo apt install ros-foxy-tf2 ros-foxy-tf2-geometry-msgs
```

## 接口说明

使用组合惯导设备的RS232接口连接MIIVII AD10 UART(232) A 接口.

## 配置参数说明
```cpp
    # 将所有参数放在一个字典中
    rtk_params = {
        'device_name' : '/dev/ttyUART_232_A',
        'baud_rate' : 460800,
        'timeout_ms' : 10,
        'base_latitude': '31.96',  #基点纬度
        'base_longtitude':'117.38',  #基点经度
        
        'local_topic_name' : 'localization/rtk_info',
        'local_frame_id' : 'rtk_link',
        'local_publish_rate' : 10.0,  # 频率为10Hz
        'imu_topic_name' : 'imu/pose',
        'imu_frame_id' : 'imu_link',
        'imu_publish_rate' : 50.0,  # 频率为50Hz
        'gnss_topic_name' : 'gnss/pose',
        'gnss_frame_id' : 'world_link',
        'gnss_publish_rate' : 10.0,  # 频率为10Hz
        'enable_debug_log': False,
        
        # info_str 文件保存配置 (新增调试功能)
        'enable_info_str_save': True,        # 启用info_str保存功能
        'info_str_save_dir': '/tmp/rtk_logs', # 保存目录
    }
```

## info_str 文件保存功能说明

为了满足调试需求，新增了将原始 `info_str` 数据保存到文件的功能：

### 功能特性
- **时间编码文件名**: 自动生成基于时间戳的文件名，格式为 `rtk_info_YYYYMMDD_HHMMSS.log`
- **可配置保存目录**: 通过 `info_str_save_dir` 参数指定保存路径
- **完整数据记录**: 记录程序运行期间的所有 `info_str` 数据
- **原始数据保存**: 直接保存原始的 `info_str` 数据，不添加额外的时间戳
- **自动文件管理**: 程序启动时创建文件，程序结束时自动关闭文件

### 配置参数
- `enable_info_str_save`: 是否启用文件保存功能 (默认: false)
- `info_str_save_dir`: 文件保存目录 (默认: "/tmp/rtk_logs")

### 使用示例
```bash
# 使用带日志功能的配置文件启动节点
ros2 run rtk rtk_node --ros-args --params-file src/localization/rtk/config/rtk_with_logging.yaml
```

### 日志文件格式
```
$GIAVP,2345,123456.789,45.67,1.23,-0.45,31.960000,117.380000,50.5,0.1,0.2,0.0,1.5,12,8,4,0.0,0.15,0.05,0.02,9.81,-0.1,0.2,0.05
$GIAVP,2345,123456.889,45.68,1.24,-0.46,31.960001,117.380001,50.6,0.1,0.2,0.0,1.5,12,8,4,0.0,0.16,0.05,0.02,9.82,-0.1,0.2,0.05
```
