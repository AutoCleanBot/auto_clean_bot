#!/bin/bash

source install/setup.bash
# Run the lidar node
ros2 launch gmsl_ros2  gmsl_launch.py
# ros2 launch miivii_gmsl_camera single.launch