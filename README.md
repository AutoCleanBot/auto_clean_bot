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
## 项目基本坐标系定义

- 定位以及其他均采用的是东北天坐标系:
  - 北向（North）对应 y 轴正方向
  - 东向（East）对应 x 轴正方向
- 角度定义：
  - 0 弧度：指向正北方向
  - π/2 弧度（90度）：指向正东方向
  - π 弧度（180度）：指向正南方向
  - -π/2 弧度（-90度）：指向正西方向