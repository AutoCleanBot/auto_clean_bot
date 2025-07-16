import matplotlib
matplotlib.use('Agg')

import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import os
import shutil

def densify_path(df, max_distance=0.1):
    print("Step 1: Densifying path...")
    east = df['east'].to_numpy()
    north = df['north'].to_numpy()
    distances = np.zeros(len(east))
    distances[1:] = np.sqrt(np.diff(east)**2 + np.diff(north)**2)
    cumulative_distances = np.cumsum(distances)
    total_distance = cumulative_distances[-1]
    
    if total_distance == 0: return df.copy()
    
    num_points = int(np.ceil(total_distance / max_distance)) + 1
    new_distances = np.linspace(0, total_distance, num_points)
    interpolated_data = {}
    
    yaw_column_name = next((col for col in ['yaw', 'heading'] if col in df.columns), None)
    if yaw_column_name:
        yaw_rad = np.deg2rad(df[yaw_column_name].to_numpy())
        unwrapped_yaw = np.unwrap(yaw_rad)
        yaw_interpolator = interp1d(cumulative_distances, unwrapped_yaw, kind='cubic', fill_value="extrapolate")
        new_unwrapped_yaw = yaw_interpolator(new_distances)
        new_yaw_rad = (new_unwrapped_yaw + np.pi) % (2 * np.pi) - np.pi
        interpolated_data[yaw_column_name] = np.rad2deg(new_yaw_rad)

    for column in df.columns:
        if column == yaw_column_name: continue
        if column.lower() in ['east', 'north', 'longitude', 'latitude', 'longtitude']:
            interpolator = interp1d(cumulative_distances, df[column].to_numpy(), kind='cubic', fill_value="extrapolate")
        else:
            interpolator = interp1d(cumulative_distances, df[column].to_numpy(), kind='linear', fill_value="extrapolate")
        interpolated_data[column] = interpolator(new_distances)
        
    result_df = pd.DataFrame(interpolated_data, columns=df.columns)
    
    for column, dtype in df.dtypes.items():
        if column in result_df: result_df[column] = result_df[column].astype(dtype)
    return result_df

def smooth_and_recalculate(df, savgol_window=101, savgol_polyorder=3):
    print("Step 2: Smoothing path coordinates with Savitzky-Golay filter...")
    if len(df) <= savgol_window:
        print(f"Warning: Not enough data points ({len(df)}) for Savgol window ({savgol_window}). Skipping smoothing.")
        return df
    if savgol_window % 2 == 0: savgol_window += 1

    smooth_df = df.copy()
    
    # 修正点: 显式转换为numpy数组
    east_raw = df['east'].to_numpy()
    north_raw = df['north'].to_numpy()
    
    smooth_df['east'] = savgol_filter(east_raw, savgol_window, savgol_polyorder)
    smooth_df['north'] = savgol_filter(north_raw, savgol_window, savgol_polyorder)
    
    print("Step 3: Recalculating derivatives (yaw, curvature) on the smoothed path...")
    east = smooth_df['east'].to_numpy()
    north = smooth_df['north'].to_numpy()
    
    dx, dy = np.gradient(east), np.gradient(north)
    yaw_rad = np.arctan2(dx, dy)
    yaw_column_name = next((col for col in ['yaw', 'heading'] if col in df.columns), 'yaw')
    smooth_df[yaw_column_name] = np.rad2deg(yaw_rad)

    ddx, ddy = np.gradient(dx), np.gradient(dy)
    numerator = dx * ddy - dy * ddx
    denominator = (dx**2 + dy**2)**1.5
    curvature = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator!=0)
    
    curvature_smooth_window = 21 if len(curvature) > 21 else len(curvature)
    if curvature_smooth_window > 1 and curvature_smooth_window % 2 == 0: curvature_smooth_window -= 1
    
    if curvature_smooth_window > savgol_polyorder:
         # 修正点: 确保传递的是numpy数组
         smooth_df['curvature'] = savgol_filter(curvature, curvature_smooth_window, savgol_polyorder)
    else:
        smooth_df['curvature'] = curvature
    return smooth_df

def verify_distances(df):
    if len(df) < 2: return 0, 0
    east, north = df['east'].to_numpy(), df['north'].to_numpy()
    distances = np.sqrt(np.diff(east)**2 + np.diff(north)**2)
    return np.max(distances), np.mean(distances)

def safe_file_replace(source_file, target_file):
    backup_file = target_file + '.bak'
    try:
        if os.path.exists(backup_file): os.remove(backup_file)
        if os.path.exists(target_file): os.rename(target_file, backup_file)
        shutil.move(source_file, target_file)
        print(f"\nFile replacement successful. Original file backed up as: {backup_file}")
        return True
    except Exception as e:
        print(f"\nError during file replacement: {e}")
        if os.path.exists(backup_file):
            try:
                if os.path.exists(target_file): os.remove(target_file)
                os.rename(backup_file, target_file)
                print("Original file has been restored.")
            except Exception as restore_error:
                print(f"FATAL: Failed to restore original file: {restore_error}")
        return False

def visualize_and_save_results(original_df, processed_df, output_image_file):
    print(f"\nVisualizing results and saving to {output_image_file}...")
    try: plt.style.use('seaborn-v0_8-whitegrid')
    except: plt.style.use('ggplot')
    
    fig, axes = plt.subplots(2, 1, figsize=(15, 12), dpi=150)
    
    # 确保使用numpy数组进行绘图
    original_east = original_df['east'].to_numpy()
    original_north = original_df['north'].to_numpy()
    processed_east = processed_df['east'].to_numpy()
    processed_north = processed_df['north'].to_numpy()
    
    axes[0].plot(original_east, original_north, 'o-', label='Original Path', color='salmon', markersize=2, alpha=0.7)
    axes[0].plot(processed_east, processed_north, '-', label='Processed Path', color='royalblue', linewidth=1.5)
    axes[0].set_title('Path Comparison', fontsize=16)
    axes[0].set_xlabel('East (m)', fontsize=12)
    axes[0].set_ylabel('North (m)', fontsize=12)
    axes[0].legend()
    axes[0].axis('equal')
    axes[0].grid(True)

    # 确保使用numpy数组计算曲率
    raw_east, raw_north = original_df['east'].to_numpy(), original_df['north'].to_numpy()
    dx, dy = np.gradient(raw_east), np.gradient(raw_north)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    numerator, denominator = dx * ddy - dy * ddx, (dx**2 + dy**2)**1.5
    original_curvature = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator!=0)

    # 确保使用numpy数组绘制曲率图
    processed_curvature = processed_df['curvature'].to_numpy()
    
    axes[1].plot(original_curvature, label='Original Curvature', color='salmon', alpha=0.7)
    axes[1].plot(processed_curvature, label='Processed Curvature', color='royalblue', linewidth=1.5)
    axes[1].set_title('Curvature Profile Comparison', fontsize=16)
    axes[1].set_xlabel('Point Index', fontsize=12)
    axes[1].set_ylabel('Curvature (1/m)', fontsize=12)
    axes[1].legend()
    axes[1].set_ylim(-0.5, 0.5)
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(output_image_file)
    plt.close(fig)
    print(f"Image successfully saved to {output_image_file}")

def main():
    INPUT_FILE = 'local_record_1.csv'
    TEMP_FILE = 'temp_processed_path.csv'
    OUTPUT_IMAGE = 'processing_result.png'
    DENSIFY_MAX_DISTANCE, SAVGOL_WINDOW, SAVGOL_POLYORDER = 0.1, 101, 3
    
    try:
        print(f"Loading data from {INPUT_FILE}...")
        df = pd.read_csv(INPUT_FILE)
        
        max_dist_orig, mean_dist_orig = verify_distances(df)
        print(f"\n--- Original Path Statistics ---")
        print(f"Number of points: {len(df)}")
        print(f"Max distance between points: {max_dist_orig:.3f} m")
        
        dense_df = densify_path(df, max_distance=DENSIFY_MAX_DISTANCE)
        processed_df = smooth_and_recalculate(dense_df, savgol_window=SAVGOL_WINDOW, savgol_polyorder=SAVGOL_POLYORDER)
        
        max_dist_proc, mean_dist_proc = verify_distances(processed_df)
        print(f"\n--- Processed Path Statistics ---")
        print(f"Number of points: {len(processed_df)}")
        print(f"Average distance between points: {mean_dist_proc:.3f} m")
        
        processed_df.to_csv(TEMP_FILE, index=False, float_format='%.8f', columns=df.columns)
        safe_file_replace(TEMP_FILE, INPUT_FILE)
        print(f"\nProcessed data saved to {TEMP_FILE}. You can now use this file.")
        
        visualize_and_save_results(df, processed_df, OUTPUT_IMAGE)
        
    except FileNotFoundError:
        print(f"Error: Input file '{INPUT_FILE}' not found.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()