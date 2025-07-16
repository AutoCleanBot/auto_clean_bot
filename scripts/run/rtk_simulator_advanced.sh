#!/bin/bash

# RTK 模拟器高级启动脚本
# 支持参数配置

# 默认参数
CSV_FILE="~/auto_clean_bot/path/local_record_2.csv"
FREQUENCY="10.0"
NOISE_MIN="1.0"
NOISE_MAX="5.0"
LOOP="true"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--file)
            CSV_FILE="$2"
            shift 2
            ;;
        -r|--rate)
            FREQUENCY="$2"
            shift 2
            ;;
        --noise-min)
            NOISE_MIN="$2"
            shift 2
            ;;
        --noise-max)
            NOISE_MAX="$2"
            shift 2
            ;;
        --no-loop)
            LOOP="false"
            shift
            ;;
        -h|--help)
            echo "RTK 模拟器启动脚本"
            echo ""
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  -f, --file FILE       指定CSV轨迹文件路径 (默认: $CSV_FILE)"
            echo "  -r, --rate RATE       指定发布频率 (默认: $FREQUENCY Hz)"
            echo "  --noise-min MIN       指定最小噪声百分比 (默认: $NOISE_MIN%)"
            echo "  --noise-max MAX       指定最大噪声百分比 (默认: $NOISE_MAX%)"
            echo "  --no-loop             禁用轨迹循环播放"
            echo "  -h, --help            显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                                    # 使用默认参数"
            echo "  $0 -f path/local_record_4.csv        # 使用指定轨迹文件"
            echo "  $0 -r 20.0 --noise-max 8.0          # 20Hz频率，最大8%噪声"
            echo "  $0 --no-loop                         # 不循环播放"
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            echo "使用 $0 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# 显示启动参数
echo "RTK 模拟器启动参数:"
echo "  CSV文件: $CSV_FILE"
echo "  发布频率: $FREQUENCY Hz"
echo "  噪声范围: $NOISE_MIN% - $NOISE_MAX%"
echo "  循环播放: $LOOP"
echo ""

# 启动节点
source install/setup.bash
ros2 launch rtk_simulator rtk_simulator.launch.py \
    csv_file_path:="$CSV_FILE" \
    publish_frequency:="$FREQUENCY" \
    noise_min_percentage:="$NOISE_MIN" \
    noise_max_percentage:="$NOISE_MAX" \
    loop_trajectory:="$LOOP"
