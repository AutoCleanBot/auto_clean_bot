std_msgs/Header header          # 时间戳和坐标系

# 基本控制信号
float32 linear_velocity          # 前进速度 (m/s)
float32 steering_angle           # 方向盘角度 (度)

# 档位控制
int8 gear                        # 当前档位：-1=倒车，0=空挡，1=前进

# 模式控制
uint8 control_mode               # 0=手动模式, 1=远程控制模式
