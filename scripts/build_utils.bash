#!/bin/bash
source install/setup.bash
colcon build --packages-select local_record --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 