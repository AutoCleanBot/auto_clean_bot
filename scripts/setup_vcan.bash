#!/bin/bash

# 检查是否已加载 vcan 模块
if ! lsmod | grep -q "^vcan"; then
    echo "Loading vcan module..."
    sudo modprobe vcan
fi

# 如果 vcan0 存在，先关闭并删除
if ip link show vcan0 > /dev/null 2>&1; then
    echo "Removing existing vcan0..."
    sudo ip link set down vcan0
    sudo ip link del vcan0
fi

# 创建并启动 vcan0
echo "Creating and starting vcan0..."
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 显示设备状态
echo "vcan0 status:"
ip link show vcan0