#!/bin/bash

source install/setup.bash
colcon build --packages-select csv_map \
     --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON