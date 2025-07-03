#!/usr/bin/env python3
"""
简单测试占用栅格地图集成功能
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from bot_msg.msg import Boundary, BoundaryPoint, LocalizationInfo
import numpy as np
import time

def main():
    rclpy.init()
    
    # 创建节点
    node = Node('simple_test_publisher')
    
    # 创建发布者
    occupancy_grid_pub = node.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
    left_boundary_pub = node.create_publisher(Boundary, '/map/left_boundary', 10)
    right_boundary_pub = node.create_publisher(Boundary, '/map/right_boundary', 10)
    localization_pub = node.create_publisher(LocalizationInfo, '/localization/rtk_info', 10)
    
    # 等待一下让发布者准备好
    time.sleep(1)
    
    # 创建测试数据
    # 1. 占用栅格地图
    grid = OccupancyGrid()
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.1
    grid.info.width = 300
    grid.info.height = 300
    grid.info.origin.position.x = -15.0
    grid.info.origin.position.y = -15.0
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0
    
    # 创建地图数据，在前方5米处添加障碍物
    data = np.zeros(grid.info.width * grid.info.height, dtype=np.int8)
    obstacle_start_x = int((5.0 + 15.0) / 0.1)
    obstacle_start_y = int((14.0 + 15.0) / 0.1)
    obstacle_end_x = int((7.0 + 15.0) / 0.1)
    obstacle_end_y = int((16.0 + 15.0) / 0.1)
    
    for x in range(obstacle_start_x, obstacle_end_x):
        for y in range(obstacle_start_y, obstacle_end_y):
            if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                index = y * grid.info.width + x
                data[index] = 100
    
    grid.data = data.tolist()
    
    # 2. 边界线
    left_boundary = Boundary()
    right_boundary = Boundary()
    left_boundary.header.stamp = node.get_clock().now().to_msg()
    left_boundary.header.frame_id = "map"
    right_boundary.header.stamp = node.get_clock().now().to_msg()
    right_boundary.header.frame_id = "map"
    
    # 左边界 (车辆左侧2米)
    for i in range(20):
        point = BoundaryPoint()
        point.east = float(i)
        point.north = 2.0
        point.up = 0.0
        left_boundary.points.append(point)
    
    # 右边界 (车辆右侧2米)
    for i in range(20):
        point = BoundaryPoint()
        point.east = float(i)
        point.north = -2.0
        point.up = 0.0
        right_boundary.points.append(point)
    
    # 3. 定位信息
    loc = LocalizationInfo()
    loc.header.stamp = node.get_clock().now().to_msg()
    loc.header.frame_id = "map"
    loc.east = 0.0
    loc.north = 0.0
    loc.up = 0.0
    loc.yaw = 0.0
    loc.pitch = 0.0
    loc.roll = 0.0
    loc.vel_speed = 0.0
    loc.rtk_status = 3
    
    # 持续发布测试数据
    print("开始持续发布测试数据...")
    try:
        while True:
            occupancy_grid_pub.publish(grid)
            left_boundary_pub.publish(left_boundary)
            right_boundary_pub.publish(right_boundary)
            localization_pub.publish(loc)

            print("发布测试数据...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("停止发布测试数据")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
