#!/bin/bash

# 自动清洁机器人系统启动器
# 支持多种启动模式和配置

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 配置文件路径
CONFIG_DIR="$(dirname "$0")/configs"
DEFAULT_CONFIG="$CONFIG_DIR/default_system.yaml"

# 全局变量
LAUNCHED_PIDS=()
LAUNCHED_TERMINALS=()

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_debug() { echo -e "${BLUE}[DEBUG]${NC} $1"; }

# 显示帮助信息
show_help() {
    cat << EOF
自动清洁机器人系统启动器

用法: $0 [选项] [模式]

模式:
    full        启动完整系统（默认）
    control     仅启动控制相关节点
    perception  仅启动感知相关节点
    planning    仅启动规划相关节点
    mapping     仅启动建图相关节点
    test        启动测试模式

选项:
    -h, --help          显示此帮助信息
    -c, --config FILE   指定配置文件
    -v, --verbose       详细输出
    -d, --dry-run       仅显示将要执行的命令，不实际执行
    --no-terminal       在后台启动，不开新终端
    --delay SECONDS     节点间启动延迟（默认2秒）

示例:
    $0 full                    # 启动完整系统
    $0 control --verbose       # 详细模式启动控制系统
    $0 test -c custom.yaml     # 使用自定义配置启动测试模式

EOF
}

# 解析命令行参数
parse_args() {
    MODE="full"
    CONFIG_FILE="$DEFAULT_CONFIG"
    VERBOSE=false
    DRY_RUN=false
    NO_TERMINAL=false
    DELAY=2
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -c|--config)
                CONFIG_FILE="$2"
                shift 2
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            -d|--dry-run)
                DRY_RUN=true
                shift
                ;;
            --no-terminal)
                NO_TERMINAL=true
                shift
                ;;
            --delay)
                DELAY="$2"
                shift 2
                ;;
            full|control|perception|planning|mapping|test|base_node|car_node)
                MODE="$1"
                shift
                ;;
            *)
                log_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# 环境检查
check_environment() {
    log_info "检查运行环境..."
    
    # 检查ROS2环境
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS2环境未设置"
        exit 1
    fi
    
    # 检查工作空间
    if [ ! -f "install/setup.bash" ]; then
        log_error "未找到install/setup.bash，请在工作空间根目录运行"
        exit 1
    fi
    
    # 检查gnome-terminal（如果需要）
    if [ "$NO_TERMINAL" = false ] && ! command -v gnome-terminal &> /dev/null; then
        log_warn "未找到gnome-terminal，将使用后台模式启动"
        NO_TERMINAL=true
    fi
    
    log_info "环境检查通过"
}

# 启动单个节点
launch_node() {
    local name="$1"
    local package="$2"
    local launch_file="$3"
    local args="$4"
    
    if [ "$VERBOSE" = true ]; then
        log_debug "启动节点: $name ($package $launch_file $args)"
    fi
    
    if [ "$DRY_RUN" = true ]; then
        echo "ros2 launch $package $launch_file $args"
        return
    fi
    
    if [ "$NO_TERMINAL" = true ]; then
        # 后台启动
        ros2 launch $package $launch_file $args &
        local pid=$!
        LAUNCHED_PIDS+=($pid)
        log_info "启动 $name (PID: $pid)"
    else
        # 新终端启动
        gnome-terminal --tab --title="$name" -- bash -c "
            source install/setup.bash
            echo '启动 $name...'
            ros2 launch $package $launch_file $args
            echo '$name 已退出，按任意键关闭终端...'
            read
        " &
        LAUNCHED_TERMINALS+=($!)
        log_info "在新终端启动 $name"
    fi
    
    sleep $DELAY
}

# 根据模式启动节点
launch_by_mode() {
    log_info "启动模式: $MODE"
    
    case $MODE in
        full)
            launch_full_system
            ;;
        control)
            launch_control_system
            ;;
        perception)
            launch_perception_system
            ;;
        planning)
            launch_planning_system
            ;;
        mapping)
            launch_mapping_system
            ;;
        test)
            launch_test_system
            ;;
        base_node)
            launch_base_node_system
            ;;
        car_node)
            launch_car_node_system
            ;;        
        *)
            log_error "未知模式: $MODE"
            exit 1
            ;;
    esac
}

# 启动完整系统
launch_full_system() {
    log_info "启动完整自动清洁机器人系统..."
    
    # 基础节点
    launch_node "RTK定位" "rtk" "rtk.launch.py" ""
    launch_node "CAN总线" "canbus_cq" "canbus.launch.py" ""
    launch_node "地图服务" "csv_map" "map_with_config.launch.py" ""
    
    # 感知节点
    launch_node "激光雷达" "lidar_driver" "lidar.launch.py" ""
    launch_node "点云预处理" "pointcloud_preprocess" "pointcloud_preprocess.launch.py" ""
    launch_node "地面滤波" "patchworkpp" "patchworkpp.launch.py" ""
    launch_node "障碍物检测" "obstacles_tracker" "obstacles_tracker.launch.py" ""
    
    # 规划控制节点
    launch_node "路径规划" "planning" "planning.launch.py" ""
    launch_node "车辆控制" "control" "control.launch.py" ""
    
    log_info "完整系统启动完成！"
}

# 启动控制系统
launch_control_system() {
    log_info "启动控制系统..."
    launch_node "RTK定位" "rtk" "rtk.launch.py" ""
    launch_node "路由服务" "routing" "routing.launch.py" ""
    launch_node "路径规划" "planning" "planning.launch.py" ""
    launch_node "车辆控制" "control" "control.launch.py" ""
    launch_node "CAN总线" "canbus_cq" "canbus.launch.py" ""
}

# 启动感知系统
launch_perception_system() {
    log_info "启动感知系统..."
    launch_node "激光雷达" "lidar_driver" "lidar.launch.py" ""
    launch_node "点云预处理" "pointcloud_preprocess" "pointcloud_preprocess.launch.py" ""
    launch_node "地面滤波" "patchworkpp" "patchworkpp.launch.py" ""
    launch_node "障碍物检测" "obstacles_tracker" "obstacles_tracker.launch.py" ""
}

# 启动规划系统
launch_planning_system() {
    log_info "启动规划系统..."
    launch_node "地图服务" "csv_map" "map_with_config.launch.py" ""
    launch_node "路径规划" "planning" "planning.launch.py" ""
    launch_node "路由服务" "routing" "routing.launch.py" ""
}

# 启动建图系统
launch_mapping_system() {
    log_info "启动建图系统..."
    launch_node "激光雷达" "lidar_driver" "lidar.launch.py" ""
    launch_node "NDT建图" "ndt_mapping" "ndt_mapping.launch.py" ""
    launch_node "RTK定位" "rtk" "rtk.launch.py" ""
}

# 启动测试系统
launch_test_system() {
    log_info "启动测试系统..."
    launch_node "控制测试" "control" "control.launch.py" ""
    launch_node "本地记录测试" "test" "local_record_test.launch.py" ""
    launch_node "路由测试" "test" "routing.launch.py" ""
    launch_node "规划测试" "test" "planning.launch.py" ""
}

launch_base_node_system() {
    log_info "启动基本节点..."
    launch_node "tf转换" "transform" "transform.launch.py" ""
    launch_node "路由节点" "routing" "routing.launch.py" ""
    launch_node "规划节点" "planning" "planning.launch.py" ""
    launch_node "点云预处理" "pointcloud_preprocess" "pointcloud_transformer.launch.py" ""
    launch_node "地面滤波" "ground_filter" "ground_filter.launch.py" ""
    launch_node "代价地图构建" "costmap_generator" "costmap_generator.launch.py" ""
    launch_node "地图节点" "csv_map" "map_with_config.launch.py" ""
    launch_node "遥控器节点" "remote_controller" "remote_controller.launch.py" ""
}


launch_car_node_system() {
    log_info "启动基本节点..."
    launch_node "tf转换" "transform" "transform.launch.py" ""
    launch_node "RTK" "rtk" "rtk.launch.py" ""
    launch_node "激光雷达" "rslidar_sdk" "start.py" ""
    launch_node "路由节点" "routing" "routing.launch.py" ""
    launch_node "规划节点" "planning" "planning.launch.py" ""
    launch_node "点云预处理" "pointcloud_preprocess" "pointcloud_transformer.launch.py" ""
    launch_node "地面滤波" "ground_filter" "ground_filter.launch.py" ""
    launch_node "代价地图构建" "costmap_generator" "costmap_generator.launch.py" ""
    launch_node "地图节点" "csv_map" "map_with_config.launch.py" ""
    launch_node "遥控器节点" "remote_controller" "remote_controller.launch.py" ""
}

# 清理函数
cleanup() {
    log_warn "正在清理进程..."
    
    # 杀死后台进程
    for pid in "${LAUNCHED_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            log_debug "终止进程 $pid"
            kill $pid
        fi
    done
    
    # 清理相关ROS2进程
    pkill -f "ros2 launch" || true
    
    log_info "清理完成"
}

# 主函数
main() {
    # 解析参数
    parse_args "$@"
    
    # 环境检查
    check_environment
    
    # 设置信号处理
    trap cleanup EXIT INT TERM
    
    # 设置工作空间环境
    source install/setup.bash
    
    # 启动系统
    launch_by_mode
    
    if [ "$DRY_RUN" = false ]; then
        log_info "系统启动完成！按Ctrl+C停止所有节点"
        
        # 等待用户中断
        if [ "$NO_TERMINAL" = true ]; then
            # 后台模式，等待所有进程
            wait
        else
            # 终端模式，等待用户输入
            read -p "按Enter键停止所有节点..."
        fi
    fi
}

# 运行主函数
main "$@"
