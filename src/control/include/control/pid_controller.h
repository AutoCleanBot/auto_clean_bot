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

    /**
     * @brief 获取积分项的值
     */
    double getIntegral() const { return integral_; }

    /**
     * @brief 设置前馈增益
     */
    void setFeedForward(double kf) { 
        this->kf_ = kf; 

    }

    /**
     * @brief 计算控制输出（带前馈）
     * @param error 当前误差
     * @param target 目标值
     * @param dt 时间间隔
     * @return 控制输出
     */
    double computeWithFeedForward(double error, double target, double dt) {
        return compute(error, dt) + this->kf_ * target;
    }

private:
    // PID参数
    double kp_;
    double ki_;
    double kd_;


    // 状态变量
    double previous_error_;
    double integral_;
    double min_integral_;
    double max_integral_;
    bool has_integral_limits_;
    // 输出限制
    double min_output_;
    double max_output_;
    bool has_output_limits_;

    // 前馈控制增益
    double kf_;
};

} // namespace control

#endif // PID_CONTROLLER_H