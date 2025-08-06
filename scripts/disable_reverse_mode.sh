#!/bin/bash

# 禁用反向移动模式脚本
# Disable Reverse Mode Script
# 
# 功能说明：
# 1. 修改 planning_params.yaml 中的 reverse_moving 为 false
# 2. 修改 control.launch.py 中的 reverse_mode 为 false
# 
# 使用方法：
# ./disable_reverse_mode.sh

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印带颜色的消息函数
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本所在目录的上级目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

print_info "项目根目录: $PROJECT_ROOT"

# 定义文件路径
PLANNING_CONFIG="$PROJECT_ROOT/src/planning/config/planning_params.yaml"
CONTROL_LAUNCH="$PROJECT_ROOT/src/control/launch/control.launch.py"

# 检查文件是否存在
if [ ! -f "$PLANNING_CONFIG" ]; then
    print_error "找不到 planning_params.yaml 文件: $PLANNING_CONFIG"
    exit 1
fi

if [ ! -f "$CONTROL_LAUNCH" ]; then
    print_error "找不到 control.launch.py 文件: $CONTROL_LAUNCH"
    exit 1
fi

print_info "开始修改配置文件..."

# 修改 planning_params.yaml
print_info "修改 planning_params.yaml 中的 reverse_moving 为 false..."
if sed -i 's/reverse_moving: true/reverse_moving: false/' "$PLANNING_CONFIG"; then
    print_info "✓ planning_params.yaml 修改成功"
else
    print_error "✗ planning_params.yaml 修改失败"
    exit 1
fi

# 修改 control.launch.py
print_info "修改 control.launch.py 中的 reverse_mode 为 False..."
if sed -i "s/'reverse_mode': True/'reverse_mode': False/" "$CONTROL_LAUNCH"; then
    print_info "✓ control.launch.py 修改成功"
else
    print_error "✗ control.launch.py 修改失败"
    exit 1
fi

print_info "所有配置修改完成！"
print_info ""
print_info "修改内容总结："
print_info "1. planning_params.yaml: reverse_moving: true → false"
print_info "2. control.launch.py: 'reverse_mode': True → False"
print_info ""
print_warning "注意：修改后的配置将在下次启动时生效" 