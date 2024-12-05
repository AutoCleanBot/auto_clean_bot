## 包依赖
`sudo apt-get install ros-foxy-camera-info-manager`
## 编译

- source /opt/ros/humble/setup.bash

- colcon build

## 使用方法

- source /opt/ros/humble/setup.bash

- source `pwd`/install/setup.bash

- ros2 launch miivii_gmsl_camera single.launch  （使用video0，分辨率为1280x720， 格式为UYVY）

## 话题名
```cpp
image_publisher_[active_camera_num] =
                this->create_publisher<sensor_msgs::msg::Image>("miivii_gmsl/image" + std::to_string(i), 10);
```

如果camera 0 是被激活了,则其对应的topic name为 `miivii_gmsl/image0`