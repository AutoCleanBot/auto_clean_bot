# 劲旅无人清扫车项目

## 构建方法

！！！基于ubuntu 20.04，ROS2 foxy

### 1.设备驱动构建

#### 1.1 激光雷达

- `inno_sw_ros2`

#### 1.2 GMSL 相机

注意 GMSL直连接口不支持热插拔，所以需要在连接相机之后，重启控制器设备

- `miivii_gmsl_ros`

  ```bash
  sudo apt install ros-foxy-camera-info-manager
  ```
- `gmsl_ros2`
  当处于其他硬件平台是使用该驱动

  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
  国内使用 rosdepc, 可以使用[一键脚本](https://github.com/fishros/install)安装

  ```
  rosdep install --from-paths src --ignore-src -r -y
  ```
  ### 2. 节点包文件
- ### 2.1 lane_detector


  - ```bash

    ```
-
