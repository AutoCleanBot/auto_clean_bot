 #!/bin/bash

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root"
    exit 1
fi

# 关闭CAN0接口
ip link set can0 down

# 加载CAN驱动模块
modprobe can
modprobe can_raw
modprobe can_dev

# 设置CAN0接口波特率（这里设置为500kbps）
ip link set can0 type can bitrate 500000

# 启动CAN0接口
ip link set can0 up

# 检查CAN0接口状态
ip -details link show can0

# 显示启动成功信息
echo "CAN0 interface has been initialized"