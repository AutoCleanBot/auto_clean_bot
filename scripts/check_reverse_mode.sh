#!/bin/bash

# 检查反向移动模式状态脚本
# Check Reverse Mode Status Script
# 
# 功能说明：
# 检查 planning_params.yaml 和 control_params.yaml 中的反向移动配置状态
# 
# 使用方法：
# ./check_reverse_mode.sh

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

print_status() {
    echo -e "${BLUE}[STATUS]${NC} $1"
}

# 获取脚本所在目录的上级目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

print_info "项目根目录: $PROJECT_ROOT"

# 定义文件路径
PLANNING_CONFIG="$PROJECT_ROOT/src/planning/config/planning_params.yaml"
CONTROL_CONFIG="$PROJECT_ROOT/src/control/config/control_params.yaml"

# 检查文件是否存在
if [ ! -f "$PLANNING_CONFIG" ]; then
    print_error "找不到 planning_params.yaml 文件: $PLANNING_CONFIG"
    exit 1
fi

if [ ! -f "$CONTROL_CONFIG" ]; then
    print_error "找不到 control_params.yaml 文件: $CONTROL_CONFIG"
    exit 1
fi

print_info "检查反向移动模式状态..."
echo ""

# 检查 planning_params.yaml 中的 reverse_moving 状态
print_status "1. planning_params.yaml 配置状态："
if grep -q "reverse_moving: true" "$PLANNING_CONFIG"; then
    print_info "   ✓ reverse_moving: true (已启用)"
elif grep -q "reverse_moving: false" "$PLANNING_CONFIG"; then
    print_warning "   ✗ reverse_moving: false (已禁用)"
else
    print_error "   ? reverse_moving 配置未找到"
fi

# 检查 planning_params.yaml 中的 planning_spd 状态
PLANNING_SPD=$(grep "planning_spd:" "$PLANNING_CONFIG" | sed 's/.*planning_spd: \([0-9.]*\).*/\1/')
if [ -n "$PLANNING_SPD" ]; then
    print_info "   ✓ planning_spd: $PLANNING_SPD"
else
    print_error "   ? planning_spd 配置未找到"
fi

echo ""

# 检查 control_params.yaml 中的 reverse_mode 状态
print_status "2. control_params.yaml 配置状态："
if grep -q "reverse_mode: true" "$CONTROL_CONFIG"; then
    print_info "   ✓ reverse_mode: true (已启用)"
elif grep -q "reverse_mode: false" "$CONTROL_CONFIG"; then
    print_warning "   ✗ reverse_mode: false (已禁用)"
else
    print_error "   ? reverse_mode 配置未找到"
fi

echo ""

# 总结状态
print_status "当前反向移动模式状态总结："
if grep -q "reverse_moving: true" "$PLANNING_CONFIG" && grep -q "reverse_mode: true" "$CONTROL_CONFIG"; then
    print_info "   ✓ 反向移动模式已完全启用"
    print_info "   ✓ 规划速度已调整为低速模式: $PLANNING_SPD m/s"
elif grep -q "reverse_moving: false" "$PLANNING_CONFIG" && grep -q "reverse_mode: false" "$CONTROL_CONFIG"; then
    print_warning "   ✗ 反向移动模式已完全禁用"
    print_info "   ✓ 规划速度为正常模式: $PLANNING_SPD m/s"
else
    print_error "   ⚠ 配置不一致，请检查配置文件"
fi

echo ""
print_info "使用方法："
print_info "  - 启用反向移动: ./enable_reverse_mode.sh"
print_info "  - 禁用反向移动: ./disable_reverse_mode.sh"
print_info "  - 检查当前状态: ./check_reverse_mode.sh" 