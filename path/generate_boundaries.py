import pandas as pd
import numpy as np
import math
from typing import Tuple, List
import matplotlib
matplotlib.use('Agg') # Ensure this is called before importing pyplot if running in non-GUI environment
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def apply_savgol_filter(data: np.ndarray, default_window: int = 21, polyorder: int = 3, max_allowable_window: int = 51) -> np.ndarray:
    """
    Applies Savitzky-Golay filter to the data with robust window size calculation.
    """
    if not isinstance(data, np.ndarray):
        data = np.array(data, dtype=float)

    data_len = len(data)
    if data_len <= polyorder:
        return data

    window = min(default_window, data_len, max_allowable_window)

    if window <= polyorder:
        window = polyorder + 1
        if window % 2 == 0: window += 1
    else:
        if window % 2 == 0:
            window -= 1
            if window <= polyorder:
                window = polyorder + 1
                if window % 2 == 0: window += 1
    
    if window > data_len: # If previous adjustments made window too large
        if data_len > polyorder:
            window = data_len
            if window % 2 == 0: window -=1
            if window <= polyorder : return data 
        else: 
            return data

    if window > polyorder and window <= data_len:
        try:
            return savgol_filter(data, window, polyorder)
        except ValueError:
            return data
    else:
        return data

def calculate_path_curvature(east: np.ndarray, north: np.ndarray) -> np.ndarray:
    """
    计算路径曲率
    使用三点法计算曲率，并进行平滑处理
    """
    curvature = np.zeros(len(east))
    n_points = len(east)
    
    if n_points < 3:
        return curvature

    for i in range(1, n_points - 1):
        x0, y0 = east[i-1], north[i-1]
        x1, y1 = east[i], north[i]
        x2, y2 = east[i+1], north[i+1]
        
        dx1, dy1 = x1 - x0, y1 - y0
        dx2, dy2 = x2 - x1, y2 - y1
        
        a_sq = dx1**2 + dy1**2
        b_sq = dx2**2 + dy2**2
        c_sq = (x2-x0)**2 + (y2-y0)**2

        if a_sq < 1e-12 or b_sq < 1e-12:
            curvature[i] = 0.0
            continue
        
        a = np.sqrt(a_sq)
        b = np.sqrt(b_sq)
        c = np.sqrt(c_sq)
        
        if a + b <= c + 1e-9 or a + c <= b + 1e-9 or b + c <= a + 1e-9 : 
            curvature[i] = 0.0
            continue

        s = (a + b + c) / 2.0
        area_arg = s*(s-a)*(s-b)*(s-c)

        if area_arg < 1e-12:
             area = 0.0
        else:
            area = np.sqrt(area_arg)
        
        denominator = a * b * c
        if denominator > 1e-9: 
            curvature[i] = 4.0 * area / denominator
            cross_product = dx1 * dy2 - dy1 * dx2 
            if cross_product < 0: 
                curvature[i] = -curvature[i]
        else:
            curvature[i] = 0.0
    
    # 端点曲率处理：在平滑前，用相邻点的值填充端点
    if n_points >= 3:
        curvature[0] = curvature[1]
        curvature[n_points-1] = curvature[n_points-2]
    
    curvature = apply_savgol_filter(curvature, default_window=21, polyorder=3, max_allowable_window=n_points)
    
    return curvature

def calculate_geometric_yaw_degrees(east: np.ndarray, north: np.ndarray) -> np.ndarray:
    """
    Calculates a smoothed geometric yaw in degrees from east and north coordinates.
    Uses padding for more stable derivative calculation at endpoints.
    """
    n_points = len(east)
    if n_points < 2:
        return np.zeros(n_points, dtype=float)

    savgol_window_derivatives = 11 
    polyorder_derivatives = 2
    pad_len = savgol_window_derivatives // 2 # e.g., 5 for window 11

    if n_points > 2 * pad_len + polyorder_derivatives : # Only pad if data is substantially longer than padding + filter needs
        east_padded = np.pad(east, (pad_len, pad_len), mode='edge')
        north_padded = np.pad(north, (pad_len, pad_len), mode='edge')

        de_padded = np.gradient(east_padded)
        dn_padded = np.gradient(north_padded)

        de_padded_smooth = apply_savgol_filter(de_padded, default_window=savgol_window_derivatives, polyorder=polyorder_derivatives, max_allowable_window=len(de_padded))
        dn_padded_smooth = apply_savgol_filter(dn_padded, default_window=savgol_window_derivatives, polyorder=polyorder_derivatives, max_allowable_window=len(dn_padded))

        de_smooth = de_padded_smooth[pad_len:-pad_len]
        dn_smooth = dn_padded_smooth[pad_len:-pad_len]
        
        # Ensure correct length after unpadding (should be guaranteed by logic)
        if len(de_smooth) != n_points: de_smooth = np.resize(de_smooth, n_points) # Basic resize if mismatch
        if len(dn_smooth) != n_points: dn_smooth = np.resize(dn_smooth, n_points)

    else: # Data too short for robust padding strategy, rely on savgol_filter's own endpoint handling
        de = np.gradient(east)
        dn = np.gradient(north)
        de_smooth = apply_savgol_filter(de, default_window=savgol_window_derivatives, polyorder=polyorder_derivatives, max_allowable_window=n_points)
        dn_smooth = apply_savgol_filter(dn, default_window=savgol_window_derivatives, polyorder=polyorder_derivatives, max_allowable_window=n_points)

    geometric_yaw_rad = np.arctan2(dn_smooth, de_smooth)
    geometric_yaw_deg = np.degrees(geometric_yaw_rad) 

    return geometric_yaw_deg

def calculate_boundary_points(east: np.ndarray, north: np.ndarray, 
                            calculated_geometric_yaw_deg: np.ndarray,
                            width: float,
                            path_curvature: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    计算边界点，考虑曲率影响，使用计算得到的几何航向角.
    """
    left_east = np.zeros_like(east)
    left_north = np.zeros_like(north)
    right_east = np.zeros_like(east)
    right_north = np.zeros_like(north)
    n_points = len(east)
    
    for i in range(n_points):
        local_width = width
        if abs(path_curvature[i]) > 1e-6: 
            adj_curvature = min(abs(path_curvature[i]), 0.5) 
            curve_factor = 1.0 / (1.0 + adj_curvature * width) 
            local_width *= curve_factor
        
        yaw_rad = math.radians(calculated_geometric_yaw_deg[i])
        norm_dx = -math.sin(yaw_rad) 
        norm_dy = math.cos(yaw_rad)
        
        left_east[i] = east[i] + norm_dx * local_width
        left_north[i] = north[i] + norm_dy * local_width
        right_east[i] = east[i] - norm_dx * local_width
        right_north[i] = north[i] - norm_dy * local_width
    
    # Window for smoothing boundary coordinates.
    # Consider making this window smaller if upstream (yaw, curvature) is very smooth.
    boundary_smooth_window = 11 
    boundary_smooth_polyorder = 3

    left_east = apply_savgol_filter(left_east, default_window=boundary_smooth_window, polyorder=boundary_smooth_polyorder, max_allowable_window=n_points)
    left_north = apply_savgol_filter(left_north, default_window=boundary_smooth_window, polyorder=boundary_smooth_polyorder, max_allowable_window=n_points)
    right_east = apply_savgol_filter(right_east, default_window=boundary_smooth_window, polyorder=boundary_smooth_polyorder, max_allowable_window=n_points)
    right_north = apply_savgol_filter(right_north, default_window=boundary_smooth_window, polyorder=boundary_smooth_polyorder, max_allowable_window=n_points)
    
    return left_east, left_north, right_east, right_north

def generate_boundaries(input_file: str, lane_width: float = 1.75):
    """
    生成车道边界线
    """
    try:
        df = pd.read_csv(input_file)
        
        required_columns = ['east', 'north'] 
        if not all(col in df.columns for col in required_columns):
            raise ValueError("Input file must contain at least 'east' and 'north' columns")
        
        east = df['east'].to_numpy()
        north = df['north'].to_numpy()

        if len(east) < 3: # Need at least 3 for curvature, more for robust smoothing
            print("Warning: Not enough data points in the input file for robust boundary generation.")
            # Decide if to proceed with potentially poor results or return None
            if len(east) < 2:
                 print("Error: Less than 2 data points. Cannot generate boundaries.")
                 return None, None, None


        path_curv = calculate_path_curvature(east, north)
        geometric_yaw_for_boundaries_deg = calculate_geometric_yaw_degrees(east, north)

        left_east, left_north, right_east, right_north = calculate_boundary_points(
            east, north, geometric_yaw_for_boundaries_deg, lane_width, path_curv
        )
        
        left_df = pd.DataFrame({
            'east': left_east,
            'north': left_north,
            'yaw': geometric_yaw_for_boundaries_deg 
        })
        
        right_df = pd.DataFrame({
            'east': right_east,
            'north': right_north,
            'yaw': geometric_yaw_for_boundaries_deg 
        })
        
        input_name = input_file.rsplit('.', 1)[0]
        left_output = f"{input_name}_left_boundary.csv"
        right_output = f"{input_name}_right_boundary.csv"
        
        left_df.to_csv(left_output, index=False, float_format='%.8f')
        right_df.to_csv(right_output, index=False, float_format='%.8f')
        
        print(f"Successfully generated boundary files:")
        print(f"Left boundary: {left_output}")
        print(f"Right boundary: {right_output}")
        
        center_df_for_plot = pd.DataFrame({
            'east': east,
            'north': north,
            'yaw': geometric_yaw_for_boundaries_deg
        })

        return left_df, right_df, center_df_for_plot
        
    except Exception as e:
        print(f"Error generating boundaries: {str(e)}")
        import traceback
        traceback.print_exc() 
        return None, None, None


def plot_boundaries(center_df_with_geometric_yaw: pd.DataFrame, left_df: pd.DataFrame, 
                   right_df: pd.DataFrame, output_file: str = 'boundaries_plot.png'):
    """
    绘制中心线和边界线.
    """
    try:
        plt.figure(figsize=(15, 10))
        
        center_east = center_df_with_geometric_yaw['east'].to_numpy()
        center_north = center_df_with_geometric_yaw['north'].to_numpy()
        center_yaw_deg = center_df_with_geometric_yaw['yaw'].to_numpy() 

        left_east = left_df['east'].to_numpy()
        left_north = left_df['north'].to_numpy()
        right_east = right_df['east'].to_numpy()
        right_north = right_df['north'].to_numpy()
        
        plt.plot(right_east, right_north, 'r--', label='Right Boundary', linewidth=1.5)
        plt.plot(center_east, center_north, 'b-', label='Center Line', linewidth=2)
        plt.plot(left_east, left_north, 'g--', label='Left Boundary', linewidth=1.5)
        
        arrow_step = max(1, len(center_east) // 20) 
        if arrow_step == 0 and len(center_east) > 0: arrow_step = 1

        # Calculate plot span for dynamic arrow sizing
        if len(center_east) > 1:
            x_min, x_max = np.min(center_east), np.max(center_east)
            y_min, y_max = np.min(center_north), np.max(center_north)
            all_east = np.concatenate([center_east, left_east, right_east])
            all_north = np.concatenate([center_north, left_north, right_north])
            plot_span_x = np.ptp(all_east) if len(all_east) > 1 else 1.0
            plot_span_y = np.ptp(all_north) if len(all_north) > 1 else 1.0
            plot_span = max(plot_span_x, plot_span_y, 1.0) # Avoid zero span
        else:
            plot_span = 1.0

        arrow_length_scale = 0.015 # Relative to plot span
        arrow_len = max(0.1, plot_span * arrow_length_scale) 
        head_w = arrow_len * 0.4
        head_l = arrow_len * 0.5


        for i in range(0, len(center_east), arrow_step):
            yaw_rad = math.radians(center_yaw_deg[i]) 
            dx_arrow = arrow_len * math.cos(yaw_rad) 
            dy_arrow = arrow_len * math.sin(yaw_rad)

            plt.arrow(center_east[i], center_north[i], dx_arrow, dy_arrow,
                     head_width=head_w, head_length=head_l, fc='blue', ec='blue', alpha=0.6, length_includes_head=True)
        
        plt.title('Path Boundaries with Driving Direction (using Geometric Yaw)')
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal') 
        
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close() 
        
        print(f"Plot saved as: {output_file}")
        
    except Exception as e:
        print(f"Error generating plot: {str(e)}")
        import traceback
        traceback.print_exc()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate path boundaries from trajectory.')
    parser.add_argument('input_file', type=str, help='Path to the input trajectory CSV file.')
    parser.add_argument('--width', type=float, default=1.75,
                        help='Half of the lane width (distance from center to one boundary). Default: 1.75m.')
    parser.add_argument('--plot', action='store_true',
                        help='Generate and save a plot of the boundaries.')
    
    args = parser.parse_args()
    
    try:
        left_df, right_df, center_df_for_plot = generate_boundaries(args.input_file, args.width)
        
        if left_df is not None and right_df is not None and center_df_for_plot is not None and args.plot:
            plot_boundaries(center_df_for_plot, left_df, right_df, 
                            output_file=f"{args.input_file.rsplit('.', 1)[0]}_boundaries_plot.png")
            
    except FileNotFoundError:
        print(f"Error: Input file '{args.input_file}' not found.")
    except pd.errors.EmptyDataError:
        print(f"Error: Input file '{args.input_file}' is empty or not a valid CSV.")
    except ValueError as ve: 
        print(f"ValueError: {str(ve)}")
    except Exception as e:
        print(f"An unexpected error occurred in main: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()