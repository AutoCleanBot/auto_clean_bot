    def control_cmd_callback(self, msg):
        """控制命令回调"""
        # 如果已经收到当前测试点的控制命令，忽略额外的命令
        if self.received_control_cmd:
            return
            
        self.control_cmd = msg
        self.received_control_cmd = True
        
        # 获取当前测试点
        if self.test_idx >= len(self.test_points):
            self.get_logger().warn(f"测试索引超出范围: {self.test_idx} >= {len(self.test_points)}")
            return
            
        test_point = self.test_points[self.test_idx]
        
        # 记录结果
        result = {
            'test_idx': self.test_idx,
            'description': test_point['description'],
            'east': test_point['east'],
            'north': test_point['north'],
            'yaw': test_point['yaw'],
            'traj_idx': test_point['traj_idx'],
            'traj_east': self.trajectory[test_point['traj_idx']]['east'],
            'traj_north': self.trajectory[test_point['traj_idx']]['north'],
            'traj_yaw': self.trajectory[test_point['traj_idx']]['yaw'],
            'traj_curvature': self.trajectory[test_point['traj_idx']]['curvature'],
            'steer_angle': msg.steer_angle,
            'speed': msg.speed
        }
        
        self.results.append(result)
        self.get_logger().info(f'已接收控制命令: 转向角={msg.steer_angle:.2f}度, 速度={msg.speed:.2f}m/s')
        
        # 移动到下一个测试点
        self.test_idx += 1
        
    def timer_callback(self):
        """定时器回调，发布轨迹和定位消息"""
        # 首先发布轨迹
        self.publish_trajectory()
        
        # 如果尚未接收到上一个控制命令的响应，等待
        if not self.received_control_cmd and self.test_idx > 0:
            self.get_logger().debug('等待控制命令响应...')
            return
            
        # 如果收到了响应，准备处理下一个测试点
        if self.received_control_cmd:
            self.received_control_cmd = False
            
        # 发布定位
        self.publish_localization() 