#!/usr/bin/env python3
"""
测试边界线生成修复效果
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from generate_boundaries import generate_boundaries

def test_boundary_generation():
    """测试边界线生成"""
    
    # 测试文件列表
    test_files = [
        'local_record_1.csv',
        'local_record_2.csv', 
        'local_record_5.csv'
    ]
    
    for test_file in test_files:
        if os.path.exists(test_file):
            print(f"\n=== 测试文件: {test_file} ===")
            
            try:
                left_df, right_df, center_df = generate_boundaries(test_file, lane_width=1.75)
                
                if left_df is not None and right_df is not None:
                    print(f"✓ 成功生成边界线")
                    print(f"  - 左边界点数: {len(left_df)}")
                    print(f"  - 右边界点数: {len(right_df)}")
                    
                    # 检查端点距离
                    left_start = (left_df.iloc[0]['east'], left_df.iloc[0]['north'])
                    left_end = (left_df.iloc[-1]['east'], left_df.iloc[-1]['north'])
                    right_start = (right_df.iloc[0]['east'], right_df.iloc[0]['north'])
                    right_end = (right_df.iloc[-1]['east'], right_df.iloc[-1]['north'])
                    
                    import numpy as np
                    left_closure = np.sqrt((left_end[0] - left_start[0])**2 + (left_end[1] - left_start[1])**2)
                    right_closure = np.sqrt((right_end[0] - right_start[0])**2 + (right_end[1] - right_start[1])**2)
                    
                    print(f"  - 左边界闭合距离: {left_closure:.2f}m")
                    print(f"  - 右边界闭合距离: {right_closure:.2f}m")
                    
                else:
                    print(f"✗ 生成边界线失败")
                    
            except Exception as e:
                print(f"✗ 处理失败: {str(e)}")
        else:
            print(f"⚠ 文件不存在: {test_file}")

if __name__ == "__main__":
    test_boundary_generation()
