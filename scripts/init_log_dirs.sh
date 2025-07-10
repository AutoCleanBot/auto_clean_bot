#!/bin/bash

# 设置颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# 主目录路径
WORKSPACE_DIR=~/auto_clean_bot
LOG_DIR=${WORKSPACE_DIR}/running_logs
CONTROL_LOG_DIR=${LOG_DIR}/control_log
OTHER_LOGS_DIR=${LOG_DIR}/other_logs

# 检查并创建主工作空间目录
if [ ! -d "$WORKSPACE_DIR" ]; then
    log_warn "工作空间目录 $WORKSPACE_DIR 不存在，创建中..."
    mkdir -p "$WORKSPACE_DIR"
    log_info "工作空间目录已创建"
else
    log_info "工作空间目录已存在"
fi

# 检查并创建日志主目录
if [ ! -d "$LOG_DIR" ]; then
    log_warn "日志主目录 $LOG_DIR 不存在，创建中..."
    mkdir -p "$LOG_DIR"
    log_info "日志主目录已创建"
else
    log_info "日志主目录已存在"
fi

# 检查并创建控制日志目录
if [ ! -d "$CONTROL_LOG_DIR" ]; then
    log_warn "控制日志目录 $CONTROL_LOG_DIR 不存在，创建中..."
    mkdir -p "$CONTROL_LOG_DIR"
    log_info "控制日志目录已创建"
else
    log_info "控制日志目录已存在"
fi

# 检查并创建其他日志目录
if [ ! -d "$OTHER_LOGS_DIR" ]; then
    log_warn "其他日志目录 $OTHER_LOGS_DIR 不存在，创建中..."
    mkdir -p "$OTHER_LOGS_DIR"
    log_info "其他日志目录已创建"
else
    log_info "其他日志目录已存在"
fi

log_info "所有日志目录检查完成"

# 设置权限，确保目录可写
chmod -R 755 "$LOG_DIR"
log_info "已设置日志目录权限"

# 设置ROS_LOG_DIR环境变量
export ROS_LOG_DIR="$OTHER_LOGS_DIR"
log_info "已设置ROS_LOG_DIR=$ROS_LOG_DIR"

# 将环境变量添加到.bashrc文件中，使其在新终端中也生效
if ! grep -q "export ROS_LOG_DIR=\"$OTHER_LOGS_DIR\"" ~/.bashrc; then
    echo "# 自动清洁机器人日志目录设置" >> ~/.bashrc
    echo "export ROS_LOG_DIR=\"$OTHER_LOGS_DIR\"" >> ~/.bashrc
    log_info "已将ROS_LOG_DIR环境变量添加到~/.bashrc文件中"
else
    log_info "ROS_LOG_DIR环境变量已存在于~/.bashrc文件中"
fi



# 提示用户重新加载环境变量
echo -e "\n要在当前终端中应用环境变量设置，请运行:"
echo -e "  ${GREEN}source ~/.bashrc${NC}"
echo -e "或者重新打开终端"

