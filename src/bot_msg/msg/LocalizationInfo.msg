std_msgs/Header header
# 位置消息
# # 经纬度坐标
float64 longtitude # 单位：度
float64 latitude # 单位：度
float64 altitude # 单位：米
# # 东北天坐标系
float64 north # 单位：米
float64 east # 单位：米
float64 up # 单位：米
# 姿态消息
float32 yaw # 单位：度
float32 pitch # 单位：度
float32 roll # 单位：度

# 速度消息
float32 vel_speed #水平速度 m/s
float32 vel_north #北向速度 m/s
float32 vel_east #东向速度 m/s
float32 vel_up #上向速度 m/s
# 加速度消息
float32 acc_x #x轴加速度 m/s^2
float32 acc_y #y轴加速度 m/s^2
float32 acc_z #z轴加速度 m/s^2
# 角速度消息
float32 gyro_x #x轴角速度 degrees/s
float32 gyro_y #y轴角速度 degrees/s
float32 gyro_z #z轴角速度 degrees/s
# rtk状态
uint8 rtk_status # 0:初始化/未定位, 1:单点定位,2:RTD,3:RTK固定解,4:RTK浮点解

