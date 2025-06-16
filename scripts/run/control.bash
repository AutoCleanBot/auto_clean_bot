#!/bin/bash

source install/setup.bash
# Run the lidar node
ros2 launch control control_lqr.launch.py
# ros2 launch control control.launch.py