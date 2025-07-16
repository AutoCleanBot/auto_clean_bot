#!/bin/bash

# 测试方向稳定性改进的启动脚本

echo "=== 启动方向稳定性测试 ==="

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 创建日志目录
mkdir -p logs

# 启动各个节点
echo "启动routing节点..."
ros2 launch routing routing.launch.py > logs/routing.log 2>&1 &
ROUTING_PID=$!

sleep 2

echo "启动RTK模拟器..."
ros2 launch rtk_simulator rtk_simulator.launch.py > logs/rtk_simulator.log 2>&1 &
RTK_PID=$!

sleep 2

echo "启动地图节点..."
ros2 launch map map.launch.py > logs/map.log 2>&1 &
MAP_PID=$!

sleep 2

echo "启动规划节点（带方向稳定性改进）..."
ros2 launch planning planning.launch.py > logs/planning.log 2>&1 &
PLANNING_PID=$!

sleep 3

echo "启动方向稳定性测试器..."
python3 test_direction_stability.py > logs/stability_test.log 2>&1 &
TESTER_PID=$!

echo "所有节点已启动，开始测试..."
echo "Routing PID: $ROUTING_PID"
echo "RTK Simulator PID: $RTK_PID"
echo "Map PID: $MAP_PID"
echo "Planning PID: $PLANNING_PID"
echo "Tester PID: $TESTER_PID"

# 等待用户输入停止
echo ""
echo "按 Ctrl+C 停止测试..."

# 捕获中断信号
trap 'echo "正在停止所有节点..."; kill $ROUTING_PID $RTK_PID $MAP_PID $PLANNING_PID $TESTER_PID 2>/dev/null; exit 0' INT

# 等待
wait

echo "测试完成"
