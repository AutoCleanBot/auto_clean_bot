std_msgs/Header header
# 位置消息
# # 经纬度坐标
float64 longtitude
float64 latitude
float64 altitude
# # 东北天坐标系
float64 north
float64 east
float64 up
# 姿态消息
float32 yaw
float32 pitch
float32 roll

# 速度消息
float32 vel_speed #水平速度
float32 vel_north #北向速度
float32 vel_east #东向速度
float32 vel_up #上向速度
# 加速度消息
float32 acc_x #x轴加速度
float32 acc_y #y轴加速度
float32 acc_z #z轴加速度
# 角速度消息
float32 gyro_x #x轴角速度
float32 gyro_y #y轴角速度    
float32 gyro_z #z轴角速度
# rtk状态
uint8 rtk_status # 0:无GPS数据，1:无差分(有数据)，2:粗差分  3:精差分(差分正常)

