#!/bin/bash

source install/setup.bash
colcon build --packages-select routing --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON