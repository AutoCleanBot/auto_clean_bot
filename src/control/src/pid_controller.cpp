#include "control/pid_controller.h"

namespace control {

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp)
    , ki_(ki)
    , kd_(kd)
    , previous_error_(0.0)
    , integral_(0.0)
    , min_output_(-std::numeric_limits<double>::max())
    , max_output_(std::numeric_limits<double>::max())
    , has_output_limits_(false)
    , min_integral_(-std::numeric_limits<double>::max())
    , max_integral_(std::numeric_limits<double>::max())
    , has_integral_limits_(false)
    , kf_(0.0) {
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    reset();
}

void PIDController::setIntegralLimits(double min_integral, double max_integral) {
    min_integral_ = min_integral;
    max_integral_ = max_integral;
    has_integral_limits_ = true;
}

void PIDController::setOutputLimits(double min_output, double max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
    has_output_limits_ = true;
}

double PIDController::compute(double error, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }

    // 定义误差死区
    const double ERROR_DEADBAND = 0.05;
    
    // 增加积分增益系数（当误差持续存在时）
    const double INTEGRAL_BOOST_THRESHOLD = 1.0;  // 1秒
    static double error_duration = 0.0;
    

   // 误差变号时减小积分项
    if (error * previous_error_ < 0 && std::abs(previous_error_) > 0.1) {
        integral_ *= 0.5;
    }

    if (std::abs(error) > ERROR_DEADBAND) {
        // 计算误差持续时间
        error_duration += dt;
        
        // 如果误差持续时间超过阈值，增加积分增益
        double integral_boost = (error_duration > INTEGRAL_BOOST_THRESHOLD) ? 2.0 : 1.0;
        
        // 增强积分作用
        integral_ += error * dt * integral_boost;


    } else {
        error_duration = 0.0;
    }

        // 应用积分限制
    if (has_integral_limits_) {
        if (integral_ > max_integral_) {
            integral_ = max_integral_;
        } else if (integral_ < min_integral_) {
            integral_ = min_integral_;
        }
    }

    // 计算微分项
    double derivative = (error - previous_error_) / dt;

    // 计算PID输出
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 应用输出限制
    if (has_output_limits_) {
        if (output > max_output_) {
            output = max_output_;
        } else if (output < min_output_) {
            output = min_output_;
        }
    }

    // 更新状态
    previous_error_ = error;

    return output;
}

void PIDController::reset() {
    previous_error_ = 0.0;
    integral_ = 0.0;
}

} // namespace control 