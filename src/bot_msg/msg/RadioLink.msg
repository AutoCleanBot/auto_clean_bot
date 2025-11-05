std_msgs/Header header          # 时间戳和坐标系

# 基本控制信号
float32 linear_pct          # 前进速度 (m/s)比率 0~1
float32 steering_pct        # 方向盘角度 (度)比率 0~1
float32 brake_pct           # 刹车比率 0~1
# 档位控制
int8 gear                        # 当前档位：-1=倒车，0=空挡，1=前进

# 模式控制
int8 control_mode               # -1=手动模式, 0=远程控制模式, 1=自动驾驶

# 启停控制
int8 stop_mode                 # 1=跑，-1=停