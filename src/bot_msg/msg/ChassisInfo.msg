std_msgs/Header header
# 线控底盘消息
float32 steer_angle # 转向反馈角度
float32 speed # 车速
uint8 gear # 档位，0x00: P/N挡位, 0x01: D挡位, 0x02: R挡位
uint8 hand_brake # 手刹状态，0x00: 松开, 0x01: 拉紧
uint8 brake # 刹车踏板百分比（0 - 100%）
uint16 error_code # 故障码
uint8 error_num            # 故障数
uint8 vehicle_fault      # 整车故障 0:正常，1：故障
uint8 unmanned_button       # 无人驾驶使能 0: 无效, 1: 有效
uint8 horn                  # 喇叭状态 0: 关闭, 1: 开启
uint8 high_beam_light       # 远光灯状态 0: 关闭, 1: 开启
uint8 low_beam_light        # 近光灯状态 0: 关闭, 1: 开启
uint8 position_light        # 位置灯状态 0: 关闭, 1: 开启
uint8 reverse_light         # 倒车灯状态 0: 关闭, 1: 开启
uint8 brake_light           # 制动灯状态 0: 关闭, 1: 开启
uint8 right_turn_light      # 右转灯状态 0: 关闭, 1: 开启
uint8 left_turn_light       # 左转灯状态 0: 关闭, 1: 开启
uint8 ready_signal          # Ready信号状态 0: 无效, 1: 有效
uint8 emergency_stop_signal # 急停信号状态 0: 无效, 1: 有效
uint8 low_water_level       # 水位低警告 0: 无效, 1: 有效
uint8 battery_level         # 电池电量百分比 0-100
uint8 driving_mode          # 驾驶模式 0: 人工驾驶, 1: 遥控驾驶, 2: 自动驾驶