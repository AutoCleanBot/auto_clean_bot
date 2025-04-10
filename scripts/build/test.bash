#!/bin/bash

source install/setup.bash
colcon build --packages-select local_record_test --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install