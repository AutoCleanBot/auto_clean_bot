#!/usr/bin/env python3
"""
测试占用栅格地图集成功能的脚本
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from bot_msg.msg import Boundary, BoundaryPoint, LocalizationInfo
import numpy as np
import time

class OccupancyGridTestPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_test_publisher')
        
        # 创建发布者
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.left_boundary_pub = self.create_publisher(Boundary, '/map/left_boundary', 10)
        self.right_boundary_pub = self.create_publisher(Boundary, '/map/right_boundary', 10)
        self.localization_pub = self.create_publisher(LocalizationInfo, '/localization/rtk_info', 10)
        
        # 创建定时器
        self.timer = self.create_timer(1.0, self.publish_test_data)
        
        self.get_logger().info('占用栅格地图测试发布者已启动')
    
    def create_test_occupancy_grid(self):
        """创建测试用的占用栅格地图"""
        grid = OccupancyGrid()
        
        # 设置头信息
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        
        # 设置地图信息
        grid.info.resolution = 0.1  # 10cm分辨率
        grid.info.width = 300       # 30m宽
        grid.info.height = 300      # 30m高
        
        # 地图原点设置为车辆位置左下角
        grid.info.origin.position.x = -15.0  # 车辆在地图中心
        grid.info.origin.position.y = -15.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        # 创建地图数据 (0=自由空间, 100=障碍物, -1=未知)
        data = np.zeros(grid.info.width * grid.info.height, dtype=np.int8)
        
        # 在车辆前方5米处添加一个障碍物 (2x2米)
        obstacle_start_x = int((5.0 + 15.0) / 0.1)  # 转换为栅格坐标
        obstacle_start_y = int((14.0 + 15.0) / 0.1)
        obstacle_end_x = int((7.0 + 15.0) / 0.1)
        obstacle_end_y = int((16.0 + 15.0) / 0.1)
        
        for x in range(obstacle_start_x, obstacle_end_x):
            for y in range(obstacle_start_y, obstacle_end_y):
                if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                    index = y * grid.info.width + x
                    data[index] = 100  # 障碍物
        
        grid.data = data.tolist()
        return grid
    
    def create_test_boundaries(self):
        """创建测试用的边界线"""
        left_boundary = Boundary()
        right_boundary = Boundary()
        
        left_boundary.header.stamp = self.get_clock().now().to_msg()
        left_boundary.header.frame_id = "map"
        right_boundary.header.stamp = self.get_clock().now().to_msg()
        right_boundary.header.frame_id = "map"
        
        # 创建左边界点 (车辆左侧2米)
        for i in range(20):  # 20米长的边界
            point = BoundaryPoint()
            point.east = float(i)
            point.north = 2.0
            point.up = 0.0
            left_boundary.points.append(point)
        
        # 创建右边界点 (车辆右侧2米)
        for i in range(20):  # 20米长的边界
            point = BoundaryPoint()
            point.east = float(i)
            point.north = -2.0
            point.up = 0.0
            right_boundary.points.append(point)
        
        return left_boundary, right_boundary
    
    def create_test_localization(self):
        """创建测试用的定位信息"""
        loc = LocalizationInfo()
        loc.header.stamp = self.get_clock().now().to_msg()
        loc.header.frame_id = "map"

        # 车辆位置设置为原点
        loc.east = 0.0
        loc.north = 0.0
        loc.up = 0.0
        loc.yaw = 0.0  # 朝向东方 (使用yaw而不是heading)
        loc.pitch = 0.0
        loc.roll = 0.0
        loc.vel_speed = 0.0
        loc.rtk_status = 3  # RTK固定解

        return loc
    
    def publish_test_data(self):
        """发布测试数据"""
        # 发布占用栅格地图
        occupancy_grid = self.create_test_occupancy_grid()
        self.occupancy_grid_pub.publish(occupancy_grid)
        
        # 发布边界线
        left_boundary, right_boundary = self.create_test_boundaries()
        self.left_boundary_pub.publish(left_boundary)
        self.right_boundary_pub.publish(right_boundary)
        
        # 发布定位信息
        localization = self.create_test_localization()
        self.localization_pub.publish(localization)
        
        self.get_logger().info('已发布测试数据: 占用栅格地图、边界线和定位信息')

def main(args=None):
    rclpy.init(args=args)
    
    node = OccupancyGridTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
