#!/bin/bash

# VSCode 终端启动脚本 - 简化版
# 用法: ./scripts/vscode_launch.sh [节点名称]

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# 工作空间路径
WORKSPACE_DIR="/home/limer/auto_clean_bot"

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# 节点配置 (节点名:包名:启动文件)
declare -A NODES=(
    ["map"]="csv_map:map_with_config.launch.py"
    ["transform"]="transform:transform.launch.py"
    ["routing"]="routing:routing.launch.py"
    ["planning"]="planning:planning.launch.py"
    ["pointcloud"]="pointcloud_preprocess:pointcloud_transformer.launch.py"
    ["ground_filter"]="ground_filter:ground_filter.launch.py"
    ["costmap"]="costmap_generator:costmap_generator.launch.py"
    ["rtk"]="rtk:rtk.launch.py"
    ["rtk_sim"]="rtk_simulator:rtk_simulator_simple.launch.py"
    ["canbus"]="canbus_cq:canbus.launch.py"
)

# 预定义的节点组合
declare -A NODE_GROUPS=(
    ["basic"]="map transform routing planning"
    ["perception"]="pointcloud ground_filter costmap"
    ["planning_only"]="routing planning"
    ["mapping"]="map transform"
    ["test_basic"]="rtk_sim map transform routing planning"
    ["all"]="map transform routing planning pointcloud ground_filter costmap"
)

# 生成启动命令
generate_launch_command() {
    local node_key="$1"
    local node_config="${NODES[$node_key]}"
    
    if [ -z "$node_config" ]; then
        echo "# 错误: 未知节点 $node_key"
        return 1
    fi
    
    local package=$(echo "$node_config" | cut -d: -f1)
    local launch_file=$(echo "$node_config" | cut -d: -f2)
    
    echo "cd $WORKSPACE_DIR && source install/setup.bash && ros2 launch $package $launch_file"
}

# 显示启动命令
show_launch_commands() {
    local nodes="$1"
    
    echo -e "${CYAN}=== VSCode 终端启动命令 ===${NC}"
    echo -e "${YELLOW}请在VSCode中为每个节点新建终端标签页，然后复制粘贴对应命令:${NC}"
    echo ""
    
    local count=1
    for node in $nodes; do
        if [[ -n "${NODES[$node]}" ]]; then
            local config="${NODES[$node]}"
            local package=$(echo "$config" | cut -d: -f1)
            
            echo -e "${GREEN}终端 $count - $node ($package):${NC}"
            echo -e "${BLUE}$(generate_launch_command $node)${NC}"
            echo ""
            ((count++))
        else
            echo -e "${RED}错误: 未知节点 $node${NC}"
        fi
    done
    
    echo -e "${YELLOW}提示: 在VSCode中按 Ctrl+Shift+反引号 可以新建终端${NC}"
}

# 显示帮助信息
show_help() {
    echo "VSCode 终端启动脚本 - 简化版"
    echo ""
    echo "用法:"
    echo "  $0 [节点名称|节点组名称]"
    echo "  $0 -l|--list          # 列出所有可用节点"
    echo "  $0 -g|--groups        # 列出所有节点组"
    echo "  $0 -h|--help          # 显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 planning           # 显示规划节点启动命令"
    echo "  $0 basic              # 显示基础节点组启动命令"
    echo "  $0 test_basic         # 显示测试基础节点组启动命令（包含RTK模拟器）"
    echo "  $0 map transform      # 显示多个节点启动命令"
    echo ""
    echo "使用方法:"
    echo "1. 运行此脚本获取启动命令"
    echo "2. 在VSCode中为每个节点新建终端 (Ctrl+Shift+反引号)"
    echo "3. 复制粘贴对应的启动命令到各个终端中"
}

# 列出所有节点
list_nodes() {
    echo -e "${BLUE}可用节点:${NC}"
    for node in "${!NODES[@]}"; do
        local config="${NODES[$node]}"
        local package=$(echo "$config" | cut -d: -f1)
        echo "  $node -> $package"
    done | sort
}

# 列出节点组
list_groups() {
    echo -e "${BLUE}可用节点组:${NC}"
    for group in "${!NODE_GROUPS[@]}"; do
        local nodes="${NODE_GROUPS[$group]}"
        echo "  $group: $nodes"
    done | sort
}

# 主函数
main() {
    # 检查参数
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi
    
    case "$1" in
        -h|--help)
            show_help
            exit 0
            ;;
        -l|--list)
            list_nodes
            exit 0
            ;;
        -g|--groups)
            list_groups
            exit 0
            ;;
    esac
    
    # 收集所有要启动的节点
    local all_nodes=""
    for arg in "$@"; do
        if [[ -n "${NODES[$arg]}" ]]; then
            # 单个节点
            all_nodes="$all_nodes $arg"
        elif [[ -n "${NODE_GROUPS[$arg]}" ]]; then
            # 节点组
            all_nodes="$all_nodes ${NODE_GROUPS[$arg]}"
        else
            log_warn "未知节点或节点组: $arg"
        fi
    done
    
    if [ -n "$all_nodes" ]; then
        show_launch_commands "$all_nodes"
    else
        echo -e "${RED}没有找到有效的节点${NC}"
        echo "使用 $0 -l 查看可用节点"
        echo "使用 $0 -g 查看可用节点组"
        exit 1
    fi
}

# 运行主函数
main "$@"
