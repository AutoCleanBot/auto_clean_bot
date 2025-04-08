#!/bin/bash

source install/setup.bash
# Run the lidar node
ros2 launch canbus canbus.launch.py
