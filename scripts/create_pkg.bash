#!/bin/bash

# 检查输入参数数量
if [ "$#" -ne 2 ]; then
    echo "用法: $0 包名 创建路径"
    exit 1
fi

# 获取包名和创建路径
package_name="$1"
create_path="$2"

# 切换到用户指定的路径
cd "$create_path" || { echo "路径不存在，退出."; exit 1; }

# 创建指定的包
ros2 pkg create "$package_name" --build-type ament_cmake

