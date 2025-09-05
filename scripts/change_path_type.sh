#!/bin/bash

# 修改路径类型和边界文件脚本
# Change Path Type and Boundary Files Script
# 
# 功能说明：
# 1. 修改 planning_params.yaml 中的 path_type 为指定序号
# 2. 修改 map_config.yaml 中的 boundary_type 为指定序号（系统会自动加载对应的边界文件）
# 
# 使用方法：
# ./change_path_type.sh <序号>
# 例如：./change_path_type.sh 5

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

print_usage() {
    echo -e "${BLUE}[USAGE]${NC} $1"
}

# 获取脚本所在目录的上级目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

print_info "项目根目录: $PROJECT_ROOT"

# 定义文件路径
PLANNING_CONFIG="$PROJECT_ROOT/src/planning/config/planning_params.yaml"
MAP_CONFIG="$PROJECT_ROOT/src/map/csv_map/config/map_config.yaml"

# 检查文件是否存在
if [ ! -f "$PLANNING_CONFIG" ]; then
    print_error "找不到 planning_params.yaml 文件: $PLANNING_CONFIG"
    exit 1
fi

if [ ! -f "$MAP_CONFIG" ]; then
    print_error "找不到 map_config.yaml 文件: $MAP_CONFIG"
    exit 1
fi

# 检查参数
if [ $# -eq 0 ]; then
    print_error "请提供序号参数"
    print_usage "使用方法: $0 <序号>"
    print_usage "例如: $0 5"
    exit 1
fi

# 获取用户输入的序号
PATH_NUMBER="$1"

# 验证序号是否为数字
if ! [[ "$PATH_NUMBER" =~ ^[0-9]+$ ]]; then
    print_error "序号必须是数字: $PATH_NUMBER"
    exit 1
fi

print_info "开始修改配置文件，序号: $PATH_NUMBER"

# 修改 planning_params.yaml 中的 path_type（只修改主要的path_type，不影响cyclic_test_path_type等）
print_info "修改 planning_params.yaml 中的 path_type 为 $PATH_NUMBER..."
if sed -i "s/^[[:space:]]*path_type:[[:space:]]*[0-9]*/    path_type: $PATH_NUMBER/" "$PLANNING_CONFIG"; then
    print_info "✓ planning_params.yaml 修改成功"
else
    print_error "✗ planning_params.yaml 修改失败"
    exit 1
fi

# 修改 map_config.yaml 中的边界类型
print_info "修改 map_config.yaml 中的 boundary_type 为 $PATH_NUMBER..."
if sed -i "s/boundary_type: [0-9]*/boundary_type: $PATH_NUMBER/" "$MAP_CONFIG"; then
    print_info "✓ boundary_type 修改成功"
else
    print_error "✗ boundary_type 修改失败"
    exit 1
fi

print_info "所有配置修改完成！"
print_info ""
print_info "修改内容总结："
print_info "1. planning_params.yaml: path_type → $PATH_NUMBER"
print_info "2. map_config.yaml: boundary_type → $PATH_NUMBER"
print_info "   - 边界文件将自动加载: local_record_${PATH_NUMBER}_left_boundary.csv"
print_info "   - 边界文件将自动加载: local_record_${PATH_NUMBER}_right_boundary.csv"
print_info ""
print_warning "注意：修改后的配置将在下次启动时生效" 