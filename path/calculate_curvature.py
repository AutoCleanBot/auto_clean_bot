import pandas as pd
import numpy as np
from scipy.signal import savgol_filter

def smooth_curvature(curvature, window_length=5, polyorder=2):
    """
    使用Savitzky-Golay滤波器平滑曲率
    参数:
    curvature: 原始曲率数据
    window_length: 窗口长度（奇数）
    polyorder: 多项式阶数
    """
    return savgol_filter(curvature, window_length, polyorder)

def calculate_curvature(east, north):
    """
    计算路径曲率
    参数:
    east: 东向坐标列表
    north: 北向坐标列表
    返回:
    curvature_list: 曲率列表
    """
    curvature_list = []
    
    # 第一个点的曲率设为0
    curvature_list.append(0.0)
    
    # 对于中间的每个点，使用前一个点和后一个点计算曲率
    for i in range(1, len(east)-1):
        # 获取连续三个点
        # p0是前一个点，p1是当前点，p2是后一个点
        x0 = east[i-1] - east[i]    # p0相对于p1的东向偏移
        y0 = north[i-1] - north[i]  # p0相对于p1的北向偏移
        x2 = east[i+1] - east[i]    # p2相对于p1的东向偏移
        y2 = north[i+1] - north[i]  # p2相对于p1的北向偏移
        
        # 计算叉积的z分量
        cross_product_z = x0 * y2 - x2 * y0
        
        # 计算三边长度
        dist_p0_p1 = np.hypot(x0, y0)
        dist_p1_p2 = np.hypot(x2, y2)
        dist_p0_p2 = np.hypot(east[i+1] - east[i-1], north[i+1] - north[i-1])
        
        # 检查点是否重合
        if min(dist_p0_p1, dist_p1_p2, dist_p0_p2) < 1e-6:
            curvature_list.append(0.0)
            continue
            
        # 计算曲率大小
        area_triangle_times_2 = abs(cross_product_z)
        curvature_magnitude = 2.0 * area_triangle_times_2 / (dist_p0_p1 * dist_p1_p2 * dist_p0_p2)
        
        # 根据叉积确定曲率符号
        if cross_product_z > 0:
            signed_curvature = -curvature_magnitude  # 右转为负曲率
        elif cross_product_z < 0:
            signed_curvature = curvature_magnitude   # 左转为正曲率
        else:
            signed_curvature = 0.0
            
        curvature_list.append(signed_curvature)
    
    # 最后一个点的曲率设为0
    curvature_list.append(0.0)
    
    return curvature_list

def main():
    # 读取CSV文件
    try:
        df = pd.read_csv('local_record_2.csv')
        
        # 计算原始曲率
        curvature = calculate_curvature(df['east'].values, df['north'].values)
        
        # 平滑曲率
        smoothed_curvature = smooth_curvature(curvature)
        
        # 创建结果DataFrame
        result_df = pd.DataFrame({
            'east': df['east'],
            'north': df['north'],
            'raw_curvature': curvature,
            'smoothed_curvature': smoothed_curvature
        })
        
        # 保存结果到新的CSV文件
        output_file = 'path_curvature_result.csv'
        result_df.to_csv(output_file, index=False)
        print(f"结果已保存到 {output_file}")
        
    except FileNotFoundError:
        print("错误：找不到输入文件 local_record_5.csv")
    except Exception as e:
        print(f"处理过程中出现错误：{str(e)}")

if __name__ == "__main__":
    main() 