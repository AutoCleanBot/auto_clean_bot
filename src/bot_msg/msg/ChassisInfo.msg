std_msgs/Header header
# 线控底盘消息

float32 steer_angle # 转向角度
float32 brk_press # 刹车压力
float32 cur_speed # 当前速度
float32 soc # 电池电量
int8 gear # 当前档位
int8 direction # 当前方向
int8 vcu_mode # VCU模式
int8 controller_online_sts # 控制器在线状态
int8 ipc_online_sts # IPC在线状态
