#!/bin/bash
set -x
set -e
CURRENT_PATH=$(cd $(dirname $0); pwd)
CLIENT_SDK_PATH=${CURRENT_PATH}/src/inno_lidar_ros/src/inno_sdk/

# build cilent sdk
cd ${CLIENT_SDK_PATH}/build
shared=1 ./build_unix.sh
echo "build status $?"
cd -

# build ros
colcon build