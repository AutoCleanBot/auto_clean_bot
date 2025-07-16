#!/bin/bash

# 快速测试方向稳定性改进

echo "=== 快速测试方向稳定性改进 ==="

# 设置环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查是否有必要的路径文件
if [ ! -f "path/local_record_1.csv" ]; then
    echo "错误: 找不到路径文件 path/local_record_1.csv"
    exit 1
fi

echo "1. 启动routing服务..."
ros2 launch routing routing.launch.py &
ROUTING_PID=$!
sleep 3

echo "2. 启动RTK模拟器..."
ros2 launch rtk_simulator rtk_simulator.launch.py &
RTK_PID=$!
sleep 3

echo "3. 启动地图服务..."
ros2 launch map map.launch.py &
MAP_PID=$!
sleep 3

echo "4. 启动改进后的规划节点..."
ros2 launch planning planning.launch.py &
PLANNING_PID=$!
sleep 3

echo "5. 监控轨迹话题..."
echo "正在监控 /planning/trajectory 话题，观察方向稳定性..."
echo "按 Ctrl+C 停止测试"

# 捕获中断信号
trap 'echo "停止所有节点..."; kill $ROUTING_PID $RTK_PID $MAP_PID $PLANNING_PID 2>/dev/null; exit 0' INT

# 监控轨迹话题
ros2 topic echo /planning/trajectory --field header.stamp

echo "测试完成"
