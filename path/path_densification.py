import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import os
import shutil

def calculate_distance(x1, y1, x2, y2):
    """计算两点之间的距离"""
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

def densify_path(df, max_distance=0.2):
    """
    对路径进行加密，确保相邻点之间的距离不超过指定值
    参数:
    df: 输入的DataFrame
    max_distance: 相邻点之间的最大允许距离（米）
    返回:
    新的DataFrame，包含加密后的路径点
    """
    # 转换为numpy数组进行处理
    east = df['east'].to_numpy()
    north = df['north'].to_numpy()
    
    # 计算路径的累计距离
    distances = [0]
    total_distance = 0
    for i in range(1, len(east)):
        d = calculate_distance(east[i-1], north[i-1], east[i], north[i])
        total_distance += d
        distances.append(total_distance)
    distances = np.array(distances)
    
    # 创建新的距离序列，确保点间距不超过max_distance
    num_points = int(np.ceil(total_distance / max_distance)) + 1
    new_distances = np.linspace(0, total_distance, num_points)
    
    # 创建存储插值结果的字典
    interpolated_data = {}
    
    # 对所有列进行插值，保持原始列的顺序
    for column in df.columns:
        # 对经纬度和位置相关的列使用三次样条插值
        if column in ['east', 'north', 'longitude', 'latitude', 'longtitude']:
            interpolator = interp1d(distances, df[column].to_numpy(), kind='cubic')
        # 对其他列使用线性插值
        else:
            interpolator = interp1d(distances, df[column].to_numpy(), kind='linear')
        interpolated_data[column] = interpolator(new_distances)
    
    # 创建新的DataFrame，保持原始列的顺序
    result_df = pd.DataFrame(interpolated_data, columns=df.columns)
    
    # 确保数据类型与原始数据一致
    for column in df.columns:
        result_df[column] = result_df[column].astype(df[column].dtype)
    
    return result_df

def verify_distances(df):
    """
    验证相邻点之间的距离
    返回最大距离和平均距离
    """
    distances = []
    east_array = df['east'].to_numpy()
    north_array = df['north'].to_numpy()
    
    for i in range(1, len(df)):
        d = calculate_distance(
            east_array[i-1], north_array[i-1],
            east_array[i], north_array[i]
        )
        distances.append(d)
    return max(distances), np.mean(distances)

def safe_file_replace(source_file, target_file):
    """
    安全地替换文件，包含备份机制
    """
    # 创建备份文件
    backup_file = target_file + '.bak'
    try:
        # 如果存在旧的备份文件，先删除
        if os.path.exists(backup_file):
            os.remove(backup_file)
        
        # 将原文件重命名为备份文件
        if os.path.exists(target_file):
            os.rename(target_file, backup_file)
        
        # 将新文件移动到目标位置
        shutil.move(source_file, target_file)
        
        print(f"File replacement successful. Original file backed up as: {backup_file}")
        return True
    
    except Exception as e:
        print(f"Error during file replacement: {str(e)}")
        # 如果出错，尝试恢复原文件
        if os.path.exists(backup_file):
            try:
                if os.path.exists(target_file):
                    os.remove(target_file)
                os.rename(backup_file, target_file)
                print("Original file has been restored")
            except Exception as restore_error:
                print(f"Failed to restore original file: {str(restore_error)}")
        return False

def main():
    try:
        # 读取原始数据
        input_file = 'local_record_5.csv'
        df = pd.read_csv(input_file)
        
        # 输出原始路径的统计信息
        max_dist_original, mean_dist_original = verify_distances(df)
        print(f"Original path statistics:")
        print(f"Number of points: {len(df)}")
        print(f"Maximum distance: {max_dist_original:.3f}m")
        print(f"Average distance: {mean_dist_original:.3f}m")
        
        # 进行路径加密
        dense_df = densify_path(df, max_distance=0.2)
        
        # 输出加密后的路径统计信息
        max_dist_dense, mean_dist_dense = verify_distances(dense_df)
        print(f"\nDensified path statistics:")
        print(f"Number of points: {len(dense_df)}")
        print(f"Maximum distance: {max_dist_dense:.3f}m")
        print(f"Average distance: {mean_dist_dense:.3f}m")
        
        # 首先保存到临时文件
        temp_file = 'temp_dense_path.csv'
        dense_df.to_csv(temp_file, 
                       index=False, 
                       float_format='%.8f',  # 保持足够的精度
                       columns=df.columns)   # 保持原始列的顺序
        
        # 安全地替换原文件
        if safe_file_replace(temp_file, input_file):
            print(f"\nOriginal file {input_file} has been successfully updated with densified path")
        else:
            print(f"\nFile replacement failed, please check error messages")
        
    except FileNotFoundError:
        print(f"Error: Input file {input_file} not found")
    except Exception as e:
        print(f"Error occurred during processing: {str(e)}")

if __name__ == "__main__":
    main() 