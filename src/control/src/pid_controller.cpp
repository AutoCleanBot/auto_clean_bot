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
    , has_output_limits_(false) {
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    reset();
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

    // 计算积分项
    integral_ += error * dt;

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