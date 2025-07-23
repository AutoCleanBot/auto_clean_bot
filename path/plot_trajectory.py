#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import os
import sys
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.font_manager as fm
from pathlib import Path

# 设置中文字体支持
def setup_chinese_fonts():
    """配置中文字体支持"""
    # 尝试查找系统中的中文字体
    chinese_fonts = [
        # Linux 常见中文字体
        '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
        '/usr/share/fonts/truetype/arphic/uming.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc',
        # macOS 常见中文字体
        '/System/Library/Fonts/PingFang.ttc',
        '/Library/Fonts/Arial Unicode.ttf',
        # Windows 常见中文字体
        'C:/Windows/Fonts/msyh.ttc',
        'C:/Windows/Fonts/simsun.ttc',
        'C:/Windows/Fonts/simhei.ttf',
        # 用户自定义字体目录
        str(Path.home() / '.fonts/SimHei.ttf'),
        str(Path.home() / '.fonts/SimSun.ttc'),
        str(Path.home() / '.fonts/msyh.ttc'),
    ]
    
    # 尝试设置中文字体
    font_found = False
    for font_path in chinese_fonts:
        if os.path.exists(font_path):
            plt.rcParams['font.family'] = ['sans-serif']
            if font_path.endswith('.ttc'):
                # .ttc 文件通常包含多个字体，需要指定索引
                try:
                    font = fm.FontProperties(fname=font_path, index=0)
                    plt.rcParams['font.sans-serif'] = [font.get_name()] + plt.rcParams['font.sans-serif']
                    font_found = True
                    print(f"使用中文字体: {font_path}")
                    break
                except:
                    continue
            else:
                # .ttf 文件直接使用
                try:
                    font = fm.FontProperties(fname=font_path)
                    plt.rcParams['font.sans-serif'] = [font.get_name()] + plt.rcParams['font.sans-serif']
                    font_found = True
                    print(f"使用中文字体: {font_path}")
                    break
                except:
                    continue
    
    # 如果没有找到中文字体，尝试使用系统默认字体
    if not font_found:
        # 尝试使用 matplotlib 内置的字体
        try:
            plt.rcParams['font.sans-serif'] = ['SimHei', 'Noto Sans CJK JP', 'Noto Sans CJK SC', 
                                              'WenQuanYi Micro Hei', 'Microsoft YaHei', 
                                              'PingFang SC', 'Heiti SC', 'Source Han Sans CN', 
                                              'Source Han Sans SC', 'STHeiti', 'Arial Unicode MS'] + plt.rcParams['font.sans-serif']
            print("尝试使用系统默认中文字体")
        except:
            print("警告: 无法找到中文字体，图片中的中文可能无法正常显示")
    
    # 解决负号显示问题
    plt.rcParams['axes.unicode_minus'] = False

class SmartFormatter(argparse.HelpFormatter):
    """帮助格式化器，支持换行和更详细的帮助信息"""
    def _split_lines(self, text, width):
        if text.startswith('R|'):
            return text[2:].splitlines()
        return argparse.HelpFormatter._split_lines(self, text, width)

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='轨迹路径可视化工具',
        formatter_class=SmartFormatter
    )
    
    parser.add_argument('input_file', type=str, help='Path to the input trajectory CSV file.')
    
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='输出图像文件路径 (默认: 与输入文件同名加上"_plot.png"后缀)')
    
    parser.add_argument('--dpi', type=int, default=150,
                        help='图像DPI分辨率 (默认: 150)')
    
    parser.add_argument('--figsize', type=str, default='12,10',
                        help='图像尺寸，格式为"宽,高"，单位为英寸 (默认: "12,10")')
    
    parser.add_argument('--title', type=str, default=None,
                        help='图像标题 (默认: 使用输入文件名)')
    
    parser.add_argument('--color', type=str, default='speed',
                        help='路径颜色映射的数据列 (默认: "speed"，如果存在的话)')
    
    parser.add_argument('--no-color', action='store_true',
                        help='不使用颜色映射，使用单一颜色绘制路径')
    
    parser.add_argument('--marker', action='store_true',
                        help='在路径点上显示标记')
    
    parser.add_argument('--grid', action='store_true',
                        help='显示网格')
    
    parser.add_argument('--font', type=str, default=None,
                        help='指定字体文件路径，用于显示中文 (例如: /usr/share/fonts/truetype/wqy/wqy-microhei.ttc)')
    
    args = parser.parse_args()
    
    # 设置默认输出文件名
    if args.output is None:
        input_base = os.path.splitext(args.input_file)[0]
        args.output = f"{input_base}_plot.png"
    
    # 解析图像尺寸
    try:
        width, height = map(float, args.figsize.split(','))
        args.figsize = (width, height)
    except:
        print(f"警告: 无法解析图像尺寸 '{args.figsize}'，使用默认值 (12,10)")
        args.figsize = (12, 10)
    
    return args

def plot_trajectory(df, args):
    """绘制轨迹路径"""
    
    # 检查必要的列是否存在
    if 'east' not in df.columns or 'north' not in df.columns:
        print("错误: CSV文件必须包含'east'和'north'列")
        return False
    
    # 提取数据
    east = df['east'].to_numpy()
    north = df['north'].to_numpy()
    
    # 创建图形
    plt.figure(figsize=args.figsize, dpi=args.dpi)
    
    # 设置标题
    if args.title:
        plt.title(args.title, fontsize=16)
    else:
        base_filename = os.path.basename(args.input_file)
        plt.title(f"轨迹路径: {base_filename}", fontsize=16)
    
    # 检查是否使用颜色映射
    if not args.no_color and args.color in df.columns:
        color_data = df[args.color].to_numpy()
        
        # 创建自定义颜色映射
        colors = ['blue', 'cyan', 'yellow', 'red']
        cmap = LinearSegmentedColormap.from_list('custom_cmap', colors)
        
        # 绘制带颜色映射的路径
        scatter = plt.scatter(east, north, 
                  c=color_data, 
                  cmap=cmap, 
                  s=5 if args.marker else 0,
                  alpha=0.8)
        
        # 添加颜色条
        cbar = plt.colorbar(scatter)
        cbar.set_label(args.color)
        
        # 使用线条连接点
        plt.plot(east, north, '-', color='gray', linewidth=0.5, alpha=0.5)
        
    else:
        # 使用单一颜色绘制
        marker = 'o' if args.marker else None
        plt.plot(east, north, 
                marker=marker, 
                markersize=3 if args.marker else 0,
                linestyle='-', 
                color='royalblue', 
                linewidth=1.5)
    
    # 设置坐标轴标签
    plt.xlabel('东向 (m)', fontsize=14)
    plt.ylabel('北向 (m)', fontsize=14)
    
    # 保持横纵比例一致
    plt.axis('equal')
    
    # 添加网格
    if args.grid:
        plt.grid(True, linestyle='--', alpha=0.7)
    
    # 添加起点和终点标记
    plt.scatter(east[0], north[0], color='green', s=100, label='起点', zorder=5)
    plt.scatter(east[-1], north[-1], color='red', s=100, label='终点', zorder=5)
    
    # 添加图例
    plt.legend(loc='best')
    
    # 调整布局
    plt.tight_layout()
    
    return True

def add_info_box(df):
    """添加信息框，显示路径基本信息"""
    
    # 计算路径总长度
    if len(df) >= 2:
        east = df['east'].to_numpy()
        north = df['north'].to_numpy()
        
        # 计算相邻点之间的距离
        distances = np.sqrt(np.diff(east)**2 + np.diff(north)**2)
        total_distance = np.sum(distances)
        
        # 获取速度信息（如果存在）
        speed_info = ""
        if 'speed' in df.columns:
            avg_speed = df['speed'].mean()
            max_speed = df['speed'].max()
            speed_info = f"平均速度: {avg_speed:.2f} m/s\n最大速度: {max_speed:.2f} m/s\n"
        
        # 创建信息文本
        info_text = (
            f"路径点数: {len(df)}\n"
            f"路径总长: {total_distance:.2f} m\n"
            f"{speed_info}"
        )
        
        # 添加文本框
        plt.annotate(info_text, xy=(0.02, 0.02), xycoords='axes fraction',
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.8),
                    fontsize=10, verticalalignment='bottom')

def generate_bash_completion():
    """生成Bash自动补全脚本"""
    completion_script = """
# 为plot_trajectory.py脚本生成的Bash自动补全脚本
# 将此脚本保存为_plot_trajectory_completion.sh并通过以下方式加载:
# source _plot_trajectory_completion.sh

_plot_trajectory_completion() {
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    
    # 所有选项列表
    opts="-i --input -o --output --dpi --figsize --title --color --no-color --marker --grid --font --help"
    
    # 如果前一个参数是这些选项之一，则提供适当的补全
    case "$prev" in
        -i|--input|-o|--output|--font)
            # 文件补全
            COMPREPLY=( $(compgen -f -- "$cur") )
            return 0
            ;;
        --color)
            # 常见的数据列名称
            COMPREPLY=( $(compgen -W "speed curvature yaw heading" -- "$cur") )
            return 0
            ;;
        --dpi)
            # 常见的DPI值
            COMPREPLY=( $(compgen -W "100 150 200 300" -- "$cur") )
            return 0
            ;;
        --figsize)
            # 常见的图像尺寸
            COMPREPLY=( $(compgen -W "12,10 16,12 8,6" -- "$cur") )
            return 0
            ;;
        *)
            ;;
    esac
    
    # 如果当前输入以'-'开头，则提供选项补全
    if [[ "$cur" == -* ]]; then
        COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
        return 0
    fi
}

# 注册补全函数
complete -F _plot_trajectory_completion plot_trajectory.py
complete -F _plot_trajectory_completion ./plot_trajectory.py
complete -F _plot_trajectory_completion python3\ plot_trajectory.py
"""
    
    # 将补全脚本保存到文件
    completion_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_plot_trajectory_completion.sh")
    try:
        with open(completion_file, 'w') as f:
            f.write(completion_script)
        print(f"\n已生成Bash自动补全脚本: {completion_file}")
        print(f"要启用自动补全功能，请执行: source {completion_file}")
    except Exception as e:
        print(f"无法生成自动补全脚本: {e}")

def main():
    """主函数"""
    # 检查是否需要生成自动补全脚本
    if '--generate-completion' in sys.argv:
        generate_bash_completion()
        return
    
    # 设置中文字体支持
    setup_chinese_fonts()
        
    # 解析命令行参数
    args = parse_arguments()
    
    # 如果指定了字体文件，则使用该字体
    if args.font and os.path.exists(args.font):
        try:
            font = fm.FontProperties(fname=args.font)
            plt.rcParams['font.sans-serif'] = [font.get_name()] + plt.rcParams['font.sans-serif']
            print(f"使用指定字体: {args.font}")
        except Exception as e:
            print(f"加载指定字体失败: {e}")
    
    try:
        # 读取CSV文件
        print(f"读取轨迹文件: {args.input_file}")
        df = pd.read_csv(args.input_file)
        
        # 绘制轨迹
        if plot_trajectory(df, args):
            # 添加信息框
            add_info_box(df)
            
            # 保存图像
            plt.savefig(args.output)
            print(f"图像已保存到: {args.output}")
            
            # 显示图像
            plt.close()
        
    except FileNotFoundError:
        print(f"错误: 找不到文件 '{args.input_file}'")
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    main() 
