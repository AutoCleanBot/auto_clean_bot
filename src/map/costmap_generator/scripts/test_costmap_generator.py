#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import time

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/points_no_ground', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Point cloud publisher initialized')

    def timer_callback(self):
        # 创建一个简单的点云，模拟地面上的障碍物
        # 创建一个 10m x 10m 的网格，每 0.5m 一个点
        x = np.arange(-5.0, 5.0, 0.5)
        y = np.arange(-5.0, 5.0, 0.5)
        xx, yy = np.meshgrid(x, y)
        
        # 将网格点展平为一维数组
        x_points = xx.flatten()
        y_points = yy.flatten()
        
        # 创建 z 坐标，大部分点在地面上 (z=0)，但在中心附近有一些障碍物 (z>0)
        z_points = np.zeros_like(x_points)
        
        # 添加一些障碍物
        # 1. 在 (2, 2) 处添加一个高度为 1m 的障碍物
        obstacle_idx = np.where((np.abs(x_points - 2.0) < 1.0) & (np.abs(y_points - 2.0) < 1.0))
        z_points[obstacle_idx] = 1.0
        
        # 2. 在 (-2, -2) 处添加一个高度为 0.5m 的障碍物
        obstacle_idx = np.where((np.abs(x_points + 2.0) < 0.8) & (np.abs(y_points + 2.0) < 0.8))
        z_points[obstacle_idx] = 0.5
        
        # 3. 在 (0, 0) 处添加一个高度为 1.5m 的障碍物
        obstacle_idx = np.where((np.abs(x_points) < 0.5) & (np.abs(y_points) < 0.5))
        z_points[obstacle_idx] = 1.5

        # 将点云数据打包为 PointCloud2 消息
        cloud_msg = self.create_point_cloud2(x_points, y_points, z_points)
        self.publisher_.publish(cloud_msg)
        self.get_logger().info('Published point cloud with {} points'.format(len(x_points)))

    def create_point_cloud2(self, x, y, z):
        # 创建 PointCloud2 消息
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'map'
        
        # 定义点云字段
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 创建点云数据
        cloud_msg.fields = fields
        cloud_msg.point_step = 12  # 每个点占用的字节数 (3 * float32)
        cloud_msg.row_step = cloud_msg.point_step * len(x)
        cloud_msg.height = 1
        cloud_msg.width = len(x)
        cloud_msg.is_dense = True
        
        # 将点云数据打包为二进制数据
        cloud_data = []
        for i in range(len(x)):
            cloud_data.append(struct.pack('fff', x[i], y[i], z[i]))
        
        cloud_msg.data = b''.join(cloud_data)
        
        return cloud_msg

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 