#!/bin/bash

# 自动清洁机器人系统启动脚本
# 用于启动控制测试相关的所有节点

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查ROS2环境
check_ros_env() {
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS2环境未设置，请先source ROS2环境"
        exit 1
    fi
    log_info "ROS2环境检查通过: $ROS_DISTRO"
}

# 检查工作空间
check_workspace() {
    if [ ! -f "install/setup.bash" ]; then
        log_error "未找到install/setup.bash，请确保在工作空间根目录运行"
        exit 1
    fi
    log_info "工作空间检查通过"
}

# 清理函数
cleanup() {
    log_warn "正在清理进程..."
    # 杀死所有相关的ROS2进程
    pkill -f "ros2 launch" || true
    pkill -f "control_node" || true
    pkill -f "local_record" || true
    pkill -f "routing_node" || true
    pkill -f "planning_node" || true
    log_info "清理完成"
}

# 设置信号处理
trap cleanup EXIT INT TERM

main() {
    log_info "开始启动自动清洁机器人控制测试系统..."

    # 环境检查
    check_ros_env
    check_workspace

    # 设置工作空间环境
    source install/setup.bash
    log_info "工作空间环境设置完成"

    # 启动控制节点（主终端）
    log_info "启动控制节点..."
    ros2 launch control control.launch.py &
    CONTROL_PID=$!
    sleep 2  # 等待控制节点启动

    # 检查控制节点是否正常启动
    if ! kill -0 $CONTROL_PID 2>/dev/null; then
        log_error "控制节点启动失败"
        exit 1
    fi

    # 启动其他节点（新终端）
    log_info "启动local_record_test节点..."
    gnome-terminal --tab --title="local_record_test" -- bash -c "
        source install/setup.bash
        ros2 launch test local_record_test.launch.py
        read -p '按任意键关闭...'
    " &

    sleep 1

    log_info "启动routing节点..."
    gnome-terminal --tab --title="routing" -- bash -c "
        source install/setup.bash
        ros2 launch test routing.launch.py
        read -p '按任意键关闭...'
    " &

    sleep 1

    log_info "启动planning节点..."
    gnome-terminal --tab --title="planning" -- bash -c "
        source install/setup.bash
        ros2 launch test planning.launch.py
        read -p '按任意键关闭...'
    " &

    log_info "所有节点启动完成！"
    log_info "按Ctrl+C停止所有节点"

    # 等待控制节点结束
    wait $CONTROL_PID
}

# 运行主函数
main "$@"



