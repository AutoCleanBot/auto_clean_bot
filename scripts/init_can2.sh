 #!/bin/bash

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root"
    exit 1
fi

# 关闭can2接口
ip link set can2 down

# 加载CAN驱动模块
modprobe can
modprobe can_raw
modprobe can_dev

# 设置can2接口波特率（这里设置为500kbps）
ip link set can2 type can bitrate 250000

# 启动can2接口
ip link set can2 up

# 检查can2接口状态
ip -details link show can2

# 显示启动成功信息
echo "can2 interface has been initialized"