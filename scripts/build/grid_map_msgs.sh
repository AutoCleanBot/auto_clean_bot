#!/bin/bash

# Grid Map Messages 编译脚本
# 用于编译 grid_map_msgs 及其依赖项

set -e

echo "=== Grid Map Messages 编译脚本 ==="

# 检查 grid_map 源码
if [ ! -d "src/grid_map" ]; then
    echo "错误: 未找到 grid_map 源码目录"
    echo "请先运行: cd src && git clone https://github.com/ANYbotics/grid_map.git"
    exit 1
fi

# 确保在正确分支
cd src/grid_map
if [ "$(git branch --show-current)" != "foxy-devel" ]; then
    echo "切换到 foxy-devel 分支..."
    git checkout foxy-devel
fi
cd ../..

# 设置环境
source /opt/ros/foxy/setup.bash
[ -d "install" ] && source install/setup.bash

# 编译依赖和主包
echo "编译 grid_map_cmake_helpers..."
colcon build --packages-select grid_map_cmake_helpers --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

echo "编译 grid_map_msgs..."
colcon build --packages-select grid_map_msgs --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release

echo "=== 编译完成 ==="
echo "请运行: source install/setup.bash"
