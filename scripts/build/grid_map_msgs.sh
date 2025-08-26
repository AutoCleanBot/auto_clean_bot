#!/bin/bash

# # Grid Map Messages 编译脚本
# # 用于编译 grid_map_msgs 及其依赖项

# set -e

# echo "=== Grid Map Messages 编译脚本 ==="

# # 检查 grid_map 源码
# if [ ! -d "src/grid_map" ]; then
#     echo "错误: 未找到 grid_map 源码目录"
#     echo "请先运行: cd src && git clone https://github.com/ANYbotics/grid_map.git"
#     exit 1
# fi

# # 确保在正确分支
# cd src/grid_map

# # 检查可用分支
# echo "检查可用分支..."
# git fetch --all

# # 检查是否存在 foxy-devel 分支
# if git show-ref --verify --quiet refs/heads/foxy-devel; then
#     echo "切换到 foxy-devel 分支..."
#     git checkout foxy-devel
# elif git show-ref --verify --quiet refs/remotes/origin/foxy-devel; then
#     echo "创建并切换到 foxy-devel 分支..."
#     git checkout -b foxy-devel origin/foxy-devel
# else
#     echo "警告: 未找到 foxy-devel 分支，检查可用分支:"
#     git branch -a | grep -E "(foxy|humble|galactic)" || true
    
#     # 尝试使用其他ROS2分支
#     if git show-ref --verify --quiet refs/remotes/origin/humble-devel; then
#         echo "使用 humble-devel 分支..."
#         git checkout -b humble-devel origin/humble-devel
#     elif git show-ref --verify --quiet refs/remotes/origin/galactic-devel; then
#         echo "使用 galactic-devel 分支..."
#         git checkout -b galactic-devel origin/galactic-devel
#     else
#         echo "使用默认分支 (通常兼容ROS2)..."
#         git checkout main 2>/dev/null || git checkout master 2>/dev/null || true
#     fi
# fi

# cd ../..

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
