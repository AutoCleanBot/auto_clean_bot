#!/bin/bash

source install/setup.bash
# Run the lidar node
# 因为在执行 source install/setup.bash 后，ROS 2 会把工作目录设置在工作空间根目录
ros2 launch seyond start.py \
 config_path:=src/drivers/seyond_lidar_ros/config/config.yaml 