#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cmath>
#include <limits>

namespace control {

class PIDController {
public:
    /**
     * @brief 构造函数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     */
    PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0);

    /**
     * @brief 设置PID参数
     */
    void setGains(double kp, double ki, double kd);

    /**
     * @brief 设置输出限制
     */
    void setOutputLimits(double min_output, double max_output);

    /**
     * @brief 计算控制输出
     * @param error 当前误差
     * @param dt 时间间隔
     * @return 控制输出
     */
    double compute(double error, double dt);

    /**
     * @brief 重置控制器状态
     */
    void reset();

private:
    // PID参数
    double kp_;
    double ki_;
    double kd_;

    // 状态变量
    double previous_error_;
    double integral_;

    // 输出限制
    double min_output_;
    double max_output_;
    bool has_output_limits_;
};

} // namespace control

#endif // PID_CONTROLLER_H 