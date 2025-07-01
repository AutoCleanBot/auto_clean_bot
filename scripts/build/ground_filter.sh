#!/bin/bash

source install/setup.bash
colcon build --packages-select ground_filter --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON