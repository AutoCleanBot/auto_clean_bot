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
    }
```