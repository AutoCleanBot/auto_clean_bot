import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
import argparse
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib import font_manager


class PathVisualizer:
    def __init__(self, data_file, output_file=None, arrow_density=10, label_density=2):
        """
        初始化路径可视化器
        
        参数:
            data_file: CSV文件路径
            output_file: 输出图片文件路径，如果为None则只显示不保存
            arrow_density: 航向箭头的密度，表示每隔多少个点绘制一次箭头
            label_density: 标签密度，表示在箭头点中每隔多少个添加标签
        """
        self.data = pd.read_csv(data_file)
        self.output_file = output_file
        self.arrow_density = arrow_density
        self.label_density = label_density
        self.fig = None
        self.ax = None
        
        # 设置中文字体
        self.setup_chinese_font()
    
    def setup_chinese_font(self):
        """配置中文字体显示"""
        # 尝试设置中文字体
        try:
            # 尝试使用微软雅黑
            plt.rcParams['font.family'] = ['Microsoft YaHei', 'SimHei', 'sans-serif']
            plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
        except:
            print("警告: 未能完全配置中文字体，可能会导致中文显示为方块")
    
    def setup_plot(self, figsize=(15, 12)):
        """设置绘图区域"""
        self.fig = plt.figure(figsize=figsize)
        self.ax = plt.subplot(111)
        
    def plot_paths(self):
        """绘制路径点集，而不是使用线段"""
        # 绘制当前位置路径点集 - 减小点的大小
        self.ax.scatter(self.data['cur_east'], self.data['cur_north'], c='r', s=5, alpha=0.5, label='当前路径')
        
        # 绘制最近参考点路径点集 - 减小点的大小
        self.ax.scatter(self.data['closest_east'], self.data['closest_north'], c='b', s=5, alpha=0.5, label='参考路径')
        
        # 添加起点和终点标记
        self.ax.plot(self.data['cur_east'].iloc[0], self.data['cur_north'].iloc[0], 'go', markersize=8)
        self.ax.plot(self.data['cur_east'].iloc[-1], self.data['cur_north'].iloc[-1], 'mo', markersize=8)
    
    def plot_heading_arrows(self, arrow_length=0.3):
        """绘制航向箭头和标注，正北为0度"""
        for i in range(0, len(self.data), self.arrow_density):
            # 获取当前位置
            cur_x = self.data['cur_east'].iloc[i]
            cur_y = self.data['cur_north'].iloc[i]
            closest_x = self.data['closest_east'].iloc[i]
            closest_y = self.data['closest_north'].iloc[i]
            
            # 计算当前点航向的方向向量（正北为0度，顺时针增加）
            # 正北为0度，对应y轴正方向，需要做坐标变换
            cur_yaw_rad = math.radians(90 - self.data['cur_yaw_deg'].iloc[i])
            dx_cur = arrow_length * math.cos(cur_yaw_rad)
            dy_cur = arrow_length * math.sin(cur_yaw_rad)
            
            # 计算最近点航向的方向向量
            closest_yaw_rad = math.radians(90 - self.data['closest_yaw_deg'].iloc[i])
            dx_closest = arrow_length * math.cos(closest_yaw_rad)
            dy_closest = arrow_length * math.sin(closest_yaw_rad)
            
            # 绘制当前点航向箭头
            self.ax.arrow(cur_x, cur_y, dx_cur, dy_cur, head_width=0.1, head_length=0.15, fc='green', ec='green')
            
            # 绘制最近点航向箭头
            self.ax.arrow(closest_x, closest_y, dx_closest, dy_closest, head_width=0.08, head_length=0.12, fc='purple', ec='purple')
            
            # 标注转向角度和横向误差，增加更多标注
            if i % (self.arrow_density * self.label_density) == 0:
                # 为当前点添加文字标签
                self.ax.annotate(
                    f"转向: {self.data['steer_angle_deg'].iloc[i]:.1f}°\nlat_err: {self.data['lat_error'].iloc[i]:.3f}m", 
                    xy=(cur_x, cur_y),
                    xytext=(cur_x + 0.15, cur_y + 0.15),
                    fontsize=8,
                    bbox=dict(boxstyle="round,pad=0.2", fc="yellow", alpha=0.7)
                )
                
                # 额外标注当前航向
                self.ax.annotate(
                    f"cur_yaw: {self.data['cur_yaw_deg'].iloc[i]:.1f}°", 
                    xy=(cur_x, cur_y),
                    xytext=(cur_x - 0.4, cur_y - 0.15),
                    fontsize=7,
                    bbox=dict(boxstyle="round,pad=0.2", fc="lightgreen", alpha=0.6)
                )
    
    def highlight_key_points(self):
        """突出显示关键点"""
        # 可视化转向角最大的点
        max_steer_idx = self.data['steer_angle_deg'].abs().idxmax()
        self.ax.plot(self.data['cur_east'].iloc[max_steer_idx], self.data['cur_north'].iloc[max_steer_idx], 'r*', markersize=10)
        self.ax.annotate(
            f"最大转向: {self.data['steer_angle_deg'].iloc[max_steer_idx]:.1f}°", 
            xy=(self.data['cur_east'].iloc[max_steer_idx], self.data['cur_north'].iloc[max_steer_idx]),
            xytext=(self.data['cur_east'].iloc[max_steer_idx] + 0.25, self.data['cur_north'].iloc[max_steer_idx] + 0.25),
            fontsize=9,
            arrowprops=dict(facecolor='black', shrink=0.05),
            bbox=dict(boxstyle="round,pad=0.3", fc="red", alpha=0.7)
        )
        
        # 可视化横向误差最大的点
        max_lat_err_idx = self.data['lat_error'].abs().idxmax()
        self.ax.plot(self.data['cur_east'].iloc[max_lat_err_idx], self.data['cur_north'].iloc[max_lat_err_idx], 'b*', markersize=10)
        self.ax.annotate(
            f"最大横向误差: {self.data['lat_error'].iloc[max_lat_err_idx]:.3f}m", 
            xy=(self.data['cur_east'].iloc[max_lat_err_idx], self.data['cur_north'].iloc[max_lat_err_idx]),
            xytext=(self.data['cur_east'].iloc[max_lat_err_idx] + 0.25, self.data['cur_north'].iloc[max_lat_err_idx] - 0.25),
            fontsize=9,
            arrowprops=dict(facecolor='black', shrink=0.05),
            bbox=dict(boxstyle="round,pad=0.3", fc="blue", alpha=0.7)
        )
        
        # 添加几个关键时间点的标注（开始、中间和接近结束的点）
        key_indices = [0, len(self.data) // 3, 2 * len(self.data) // 3, len(self.data) - 10]
        for idx in key_indices:
            self.ax.annotate(
                f"t={idx}帧\n航向={self.data['cur_yaw_deg'].iloc[idx]:.1f}°",
                xy=(self.data['cur_east'].iloc[idx], self.data['cur_north'].iloc[idx]),
                xytext=(self.data['cur_east'].iloc[idx] + 0.3, self.data['cur_north'].iloc[idx] + 0.2),
                fontsize=8,
                arrowprops=dict(facecolor='gray', shrink=0.05, width=1),
                bbox=dict(boxstyle="round,pad=0.2", fc="lightblue", alpha=0.7)
            )
    
    def add_legend_and_labels(self):
        """添加图例和标签"""
        # 添加自定义图例
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='r', markersize=6, alpha=0.5, label='当前路径点'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='b', markersize=6, alpha=0.5, label='参考路径点'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='g', markersize=6, label='起点'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='m', markersize=6, label='终点'),
            Patch(facecolor='green', alpha=0.8, label='当前点航向'),
            Patch(facecolor='purple', alpha=0.8, label='参考点航向'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='r', markersize=8, label='最大转向点'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='b', markersize=8, label='最大横向误差点'),
            Patch(facecolor='yellow', alpha=0.7, label='转向角和横向误差标注'),
            Patch(facecolor='lightgreen', alpha=0.6, label='航向标注')
        ]
        
        # 添加图例
        self.ax.legend(handles=legend_elements, loc='upper right')
        
        # 添加标题和轴标签
        self.ax.set_title('车辆路径与航向可视化', fontsize=16)
        self.ax.set_xlabel('East (m)', fontsize=12)
        self.ax.set_ylabel('North (m)', fontsize=12)
        self.ax.grid(True, linestyle='--', alpha=0.7)
    
    def add_statistics(self):
        """添加统计信息"""
        stats_text = (
            f"数据点数: {len(self.data)}\n"
            f"平均转向角: {self.data['steer_angle_deg'].mean():.2f}°\n"
            f"最大转向角: {self.data['steer_angle_deg'].max():.2f}°\n"
            f"最小转向角: {self.data['steer_angle_deg'].min():.2f}°\n"
            f"平均航向角: {self.data['cur_yaw_deg'].mean():.2f}°\n"
            f"平均横向误差: {self.data['lat_error'].mean():.4f}m\n"
            f"最大横向误差: {self.data['lat_error'].abs().max():.4f}m\n"
            f"最小横向误差: {self.data['lat_error'].abs().min():.4f}m\n"
            f"航向角范围: {self.data['cur_yaw_deg'].min():.1f}° ~ {self.data['cur_yaw_deg'].max():.1f}°"
        )
        
        # 放置统计文本在左上角
        self.ax.text(
            0.02, 0.98, stats_text, 
            transform=self.ax.transAxes, 
            fontsize=9,
            verticalalignment='top', 
            horizontalalignment='left',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.7)
        )
    
    def finalize_plot(self):
        """完成绘图的最后步骤"""
        # 保持纵横比例一致
        self.ax.set_aspect('equal')
        
        # 调整布局
        plt.tight_layout()
        
        # 如果指定了输出文件，保存图像
        if self.output_file:
            plt.savefig(self.output_file, dpi=300)
        
        # 显示图像
        plt.show()
    
    def visualize(self):
        """执行完整的可视化流程"""
        self.setup_plot()
        self.plot_paths()
        self.plot_heading_arrows()
        self.highlight_key_points()
        self.add_legend_and_labels()
        self.add_statistics()
        self.finalize_plot()


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='可视化车辆路径与航向')
    parser.add_argument('--data', type=str, default='control_debug_20250617_185255.csv', help='数据文件路径')
    parser.add_argument('--output', type=str, default='path_visualization.png', help='输出图像文件路径')
    parser.add_argument('--arrow-density', type=int, default=10, help='箭头密度(每隔多少个点绘制一次箭头)')
    parser.add_argument('--label-density', type=int, default=2, help='标签密度(每隔多少个箭头添加一个标签)')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    
    # 创建可视化器并执行可视化
    visualizer = PathVisualizer(
        data_file=args.data,
        output_file=args.output,
        arrow_density=args.arrow_density,
        label_density=args.label_density
    )
    visualizer.visualize() 