#!/bin/bash

source install/setup.bash
# Run the lidar node
ros2 launch miivii_gmsl_camera miivii_gmsl.launch
# ros2 launch miivii_gmsl_camera single.launch