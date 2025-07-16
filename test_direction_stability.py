#!/usr/bin/env python3
"""
测试方向稳定性改进的脚本
用于验证轨迹抖动问题是否得到解决
"""

import rclpy
from rclpy.node import Node
from bot_msg.msg import ADCTrajectory, LocalizationInfo
import time
import math

class DirectionStabilityTester(Node):
    def __init__(self):
        super().__init__('direction_stability_tester')
        
        # 订阅规划轨迹
        self.trajectory_sub = self.create_subscription(
            ADCTrajectory,
            '/planning/trajectory',
            self.trajectory_callback,
            10
        )
        
        # 订阅定位信息
        self.localization_sub = self.create_subscription(
            LocalizationInfo,
            '/localization/rtk_info',
            self.localization_callback,
            10
        )
        
        # 存储历史数据
        self.trajectory_history = []
        self.localization_history = []
        self.direction_changes = []
        
        # 统计信息
        self.total_trajectories = 0
        self.direction_reversals = 0
        self.large_index_jumps = 0
        
        # 创建定时器进行统计分析
        self.timer = self.create_timer(5.0, self.analyze_stability)
        
        self.get_logger().info("Direction Stability Tester started")
        
    def trajectory_callback(self, msg):
        """轨迹回调函数"""
        if len(msg.points) == 0:
            return
            
        current_time = time.time()
        
        # 计算轨迹的平均方向
        if len(msg.points) > 1:
            total_yaw = 0
            for point in msg.points:
                total_yaw += point.yaw
            avg_yaw = total_yaw / len(msg.points)
            
            # 存储轨迹信息
            traj_info = {
                'timestamp': current_time,
                'avg_yaw': avg_yaw,
                'point_count': len(msg.points),
                'first_point': {
                    'east': msg.points[0].east,
                    'north': msg.points[0].north,
                    'yaw': msg.points[0].yaw
                }
            }
            
            self.trajectory_history.append(traj_info)
            self.total_trajectories += 1
            
            # 检测方向变化
            if len(self.trajectory_history) > 1:
                prev_yaw = self.trajectory_history[-2]['avg_yaw']
                curr_yaw = traj_info['avg_yaw']
                
                # 计算角度差异（考虑角度环绕）
                yaw_diff = abs(self.normalize_angle(curr_yaw - prev_yaw))
                
                if yaw_diff > 90:  # 如果方向变化超过90度
                    self.direction_reversals += 1
                    self.get_logger().warn(
                        f"Direction reversal detected! Yaw change: {yaw_diff:.1f}°"
                    )
                
                self.direction_changes.append(yaw_diff)
            
            # 保持历史记录在合理范围内
            if len(self.trajectory_history) > 100:
                self.trajectory_history.pop(0)
                
    def localization_callback(self, msg):
        """定位信息回调函数"""
        current_time = time.time()
        
        loc_info = {
            'timestamp': current_time,
            'east': msg.east,
            'north': msg.north,
            'yaw': msg.yaw
        }
        
        self.localization_history.append(loc_info)
        
        # 保持历史记录在合理范围内
        if len(self.localization_history) > 100:
            self.localization_history.pop(0)
    
    def normalize_angle(self, angle):
        """角度归一化到[-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def analyze_stability(self):
        """分析方向稳定性"""
        if self.total_trajectories == 0:
            self.get_logger().info("No trajectories received yet...")
            return
            
        # 计算统计信息
        reversal_rate = (self.direction_reversals / self.total_trajectories) * 100
        
        avg_direction_change = 0
        if len(self.direction_changes) > 0:
            avg_direction_change = sum(self.direction_changes) / len(self.direction_changes)
        
        # 输出统计信息
        self.get_logger().info("=== Direction Stability Analysis ===")
        self.get_logger().info(f"Total trajectories: {self.total_trajectories}")
        self.get_logger().info(f"Direction reversals: {self.direction_reversals}")
        self.get_logger().info(f"Reversal rate: {reversal_rate:.2f}%")
        self.get_logger().info(f"Average direction change: {avg_direction_change:.2f}°")
        
        # 评估稳定性
        if reversal_rate < 5.0:
            self.get_logger().info("✓ Direction stability: GOOD")
        elif reversal_rate < 15.0:
            self.get_logger().warn("⚠ Direction stability: MODERATE")
        else:
            self.get_logger().error("✗ Direction stability: POOR")
            
        # 最近轨迹信息
        if len(self.trajectory_history) > 0:
            latest = self.trajectory_history[-1]
            self.get_logger().info(f"Latest trajectory: {latest['point_count']} points, "
                                 f"avg yaw: {latest['avg_yaw']:.1f}°")
        
        # 最近定位信息
        if len(self.localization_history) > 0:
            latest_loc = self.localization_history[-1]
            self.get_logger().info(f"Vehicle position: ({latest_loc['east']:.2f}, "
                                 f"{latest_loc['north']:.2f}), yaw: {latest_loc['yaw']:.1f}°")

def main():
    rclpy.init()
    
    tester = DirectionStabilityTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
