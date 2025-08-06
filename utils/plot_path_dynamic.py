import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
import argparse
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib import font_manager

# 确保使用交互式后端
plt.switch_backend('TkAgg')  # 或尝试 'Qt5Agg'
# 启用交互模式
plt.ion()


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
        # 读取CSV文件，确保正确处理列名中的空格
        self.data = pd.read_csv(data_file, skipinitialspace=True)

        # 检查并修复列名中可能的空格问题
        self.data.columns = [col.strip() for col in self.data.columns]

        self.output_file = output_file
        self.arrow_density = arrow_density
        self.label_density = label_density
        self.fig = None
        self.ax = None
        
        # 设置中文字体
        self.setup_chinese_font()

        # 打印列名，用于调试
        print("CSV文件列名:", self.data.columns.tolist())

        # 检查是否有包含feedback_steer_deg的列
        feedback_cols = [col for col in self.data.columns if 'feedback' in col.lower(
        ) and 'steer' in col.lower()]
        if feedback_cols:
            print("找到反馈转向列:", feedback_cols)
        else:
            print("警告: 未找到反馈转向列")
            # 尝试查看第一行数据，看看是否有类似的列
            print("数据第一行:", self.data.iloc[0].to_dict())

    def setup_chinese_font(self):
        """配置中文字体显示"""
        # 尝试设置中文字体
        try:
            # 尝试使用微软雅黑
            plt.rcParams['font.family'] = [
                'Microsoft YaHei', 'SimHei', 'sans-serif']
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
        self.ax.scatter(
            self.data['cur_east'], self.data['cur_north'], c='r', s=5, alpha=0.5, label='当前路径')

        # 绘制最近参考点路径点集 - 减小点的大小
        self.ax.scatter(
            self.data['closest_east'], self.data['closest_north'], c='b', s=5, alpha=0.5, label='参考路径')

        # 绘制目标点路径点集（如果有）- 减小点的大小
        if 'target_east' in self.data.columns and 'target_north' in self.data.columns:
            self.ax.scatter(
                self.data['target_east'], self.data['target_north'], c='g', s=5, alpha=0.5, label='目标路径')

    def plot_heading_arrows(self, arrow_length=0.3):
        """绘制航向箭头，不添加文字标注"""
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
            closest_yaw_rad = math.radians(
                90 - self.data['closest_yaw_deg'].iloc[i])
            dx_closest = arrow_length * math.cos(closest_yaw_rad)
            dy_closest = arrow_length * math.sin(closest_yaw_rad)
            
            # 绘制当前点航向箭头
            self.ax.arrow(cur_x, cur_y, dx_cur, dy_cur, head_width=0.1,
                          head_length=0.15, fc='green', ec='green')
            
            # 绘制最近点航向箭头
            self.ax.arrow(closest_x, closest_y, dx_closest, dy_closest,
                          head_width=0.08, head_length=0.12, fc='purple', ec='purple')
    
    def highlight_key_points(self):
        """突出显示关键点，不添加文字标注"""
        # 可视化转向角最大的点
        max_steer_idx = self.data['steer_angle_deg'].abs().idxmax()
        max_steer_x = self.data['cur_east'].iloc[max_steer_idx]
        max_steer_y = self.data['cur_north'].iloc[max_steer_idx]

        self.ax.plot(max_steer_x, max_steer_y, 'r*', markersize=10)
        
        # 可视化横向误差最大的点
        max_lat_err_idx = self.data['lat_error'].abs().idxmax()
        max_lat_err_x = self.data['cur_east'].iloc[max_lat_err_idx]
        max_lat_err_y = self.data['cur_north'].iloc[max_lat_err_idx]

        self.ax.plot(max_lat_err_x, max_lat_err_y, 'b*', markersize=10)
    
    def add_legend_and_labels(self):
        """添加图例和标签，优化位置避免与数据重叠"""
        # 添加自定义图例
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='r',
                   markersize=6, alpha=0.5, label='当前路径点'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='b',
                   markersize=6, alpha=0.5, label='参考路径点'),
            Patch(facecolor='green', alpha=0.8, label='当前点航向'),
            Patch(facecolor='purple', alpha=0.8, label='参考点航向'),
            Line2D([0], [0], marker='*', color='w',
                   markerfacecolor='r', markersize=8, label='最大转向点'),
            Line2D([0], [0], marker='*', color='w',
                   markerfacecolor='b', markersize=8, label='最大横向误差点'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow',
                   markersize=8, alpha=0.8, label='选中的当前点'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='cyan',
                   markersize=8, alpha=0.8, label='选中的最近点'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='magenta',
                   markersize=8, alpha=0.8, label='选中的目标点')
        ]

        # 图例固定放在右上角，使用较小的字体
        self.ax.legend(handles=legend_elements, loc='upper right',
                       framealpha=0.9, fancybox=True, shadow=True,
                       fontsize=8)
        
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

    def add_interactive_features(self):
        """添加交互式功能，允许点击查看点的详细信息"""
        # 创建一个文本框用于显示信息，放在右下角
        self.info_text = self.ax.text(
            0.98, 0.02, "", transform=self.ax.transAxes,
            bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.8),
            fontsize=9, verticalalignment='bottom', horizontalalignment='right'
        )

        # 创建一个点来标记选中的当前位置 - 减小点的大小
        self.selected_point, = self.ax.plot([], [], 'o', ms=8, color='yellow',
                                            alpha=0.8, visible=False)

        # 创建一个点来标记对应的最近点位置 - 减小点的大小
        self.selected_closest, = self.ax.plot([], [], 'o', ms=8, color='cyan',
                                              alpha=0.8, visible=False)

        # 创建一个点来标记对应的目标点位置 - 减小点的大小
        self.selected_target, = self.ax.plot([], [], 'o', ms=8, color='magenta',
                                             alpha=0.8, visible=False)

        # 连接点击事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

    def on_click(self, event):
        """处理点击事件"""
        if event.inaxes != self.ax:
            return

        # 获取点击位置
        x, y = event.xdata, event.ydata

        # 找到最近的数据点
        distances = np.sqrt((self.data['cur_east'] - x)**2 +
                            (self.data['cur_north'] - y)**2)
        idx = distances.idxmin()

        # 更新选中的当前点位置
        self.selected_point.set_data(
            self.data['cur_east'].iloc[idx],
            self.data['cur_north'].iloc[idx]
        )
        self.selected_point.set_visible(True)

        # 更新选中的最近点位置
        self.selected_closest.set_data(
            self.data['closest_east'].iloc[idx],
            self.data['closest_north'].iloc[idx]
        )
        self.selected_closest.set_visible(True)

        # 更新选中的目标点位置（如果有）
        if 'target_east' in self.data.columns and 'target_north' in self.data.columns:
            self.selected_target.set_data(
                self.data['target_east'].iloc[idx],
                self.data['target_north'].iloc[idx]
            )
            self.selected_target.set_visible(True)

        # 构建信息文本，包含更多控制相关数据
        info = (
            f"帧: {idx}\n"
            f"当前位置: ({self.data['cur_east'].iloc[idx]:.2f}, {self.data['cur_north'].iloc[idx]:.2f})\n"
            f"最近点: ({self.data['closest_east'].iloc[idx]:.2f}, {self.data['closest_north'].iloc[idx]:.2f})\n"
        )

        # 添加目标点信息（如果有）
        if 'target_east' in self.data.columns and 'target_north' in self.data.columns:
            info += f"目标点: ({self.data['target_east'].iloc[idx]:.2f}, {self.data['target_north'].iloc[idx]:.2f})\n"

        # 添加航向信息
        info += (
            f"当前航向: {self.data['cur_yaw_deg'].iloc[idx]:.2f}°\n"
            f"目标航向: {self.data['closest_yaw_deg'].iloc[idx]:.2f}°\n"
            f"横向误差: {self.data['lat_error'].iloc[idx]:.3f}m\n"
            f"转向角: {self.data['steer_angle_deg'].iloc[idx]:.2f}°\n"
        )

        # 添加纯追踪和Stanley控制角度
        if 'pursuit_control_deg' in self.data.columns:
            info += f"纯追踪角: {self.data['pursuit_control_deg'].iloc[idx]:.2f}°\n"
        if 'stanley_control_deg' in self.data.columns:
            info += f"Stanley角: {self.data['stanley_control_deg'].iloc[idx]:.2f}°\n"

        # 添加反馈转向角 - 尝试多种可能的列名
        feedback_value = None

        # 尝试直接匹配
        if ' feedback_steer_deg' in self.data.columns:  # 注意前面的空格
            feedback_value = self.data[' feedback_steer_deg'].iloc[idx]
        elif 'feedback_steer_deg' in self.data.columns:
            feedback_value = self.data['feedback_steer_deg'].iloc[idx]
        else:
            # 尝试模糊匹配
            for col in self.data.columns:
                if 'feedback' in col.lower() and 'steer' in col.lower():
                    feedback_value = self.data[col].iloc[idx]
                    print(f"使用列 '{col}' 作为反馈转向角")
                    break

        if feedback_value is not None:
            info += f"反馈转向: {feedback_value:.2f}°\n"
        else:
            # 如果仍然找不到，添加一个注释
            info += "反馈转向: 数据不可用\n"

            # 打印当前行的所有数据，帮助调试
            if idx == 0:  # 只打印第一次点击的信息，避免过多输出
                print("当前行数据:", self.data.iloc[idx].to_dict())

        # 更新信息文本
        self.info_text.set_text(info)

        # 重绘图形
        self.fig.canvas.draw_idle()

    def finalize_plot(self):
        """完成绘图的最后步骤"""
        # 保持纵横比例一致
        self.ax.set_aspect('equal')
        
        # 调整布局
        plt.tight_layout()
        
        # 确保工具栏可见，以便使用缩放功能
        self.fig.canvas.manager.set_window_title('车辆路径可视化 - 使用鼠标滚轮缩放')

    def visualize(self):
        """执行完整的可视化流程"""
        self.setup_plot()
        self.plot_paths()
        self.plot_heading_arrows()
        self.highlight_key_points()
        self.add_legend_and_labels()
        self.add_statistics()
        self.add_interactive_features()  # 添加交互功能
        self.finalize_plot()

        # 显示图形（交互模式下需要显示）
        if self.output_file:
            plt.savefig(self.output_file, dpi=300)
            print(f"图像已保存到: {self.output_file}")

        # 使用阻塞模式显示图形，确保交互功能可用
        plt.ioff()  # 关闭交互模式，以便show()会阻塞
        plt.show()  # 阻塞直到窗口关闭


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='可视化车辆路径与航向')
    parser.add_argument(
        '--data', type=str, default='control_debug_20250617_185255.csv', help='数据文件路径')
    # parser.add_argument('--output', type=str, default='path_visualization.png', help='输出图像文件路径')

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    output_file = args.data.replace('.csv', '.png')
    arrow_density = 8
    label_density = 8
    # 创建可视化器并执行可视化
    visualizer = PathVisualizer(
        data_file=args.data,
        output_file=output_file,
        arrow_density=arrow_density,
        label_density=label_density
    )
    visualizer.visualize()
