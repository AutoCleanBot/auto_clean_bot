std_msgs/Header header

bool line_valid_flag # True if the line is valid, False otherwise
float32 linelength # length of the line in meters
#数据解释：车道曲线参数,三次多项式拟合参数A,B,C,D,曲线长度length
float32 param_a
float32 param_b
float32 param_c
float32 param_d
int64 length
