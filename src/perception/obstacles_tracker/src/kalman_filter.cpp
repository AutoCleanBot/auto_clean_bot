#include "obstacles_tracker/kalman_filter.h"

namespace obstacles_tracker {

KalmanFilter::KalmanFilter(double dt, double process_noise_pos, double process_noise_vel,
                           double measurement_noise_pos, double measurement_noise_size)
    : dt_(dt), initialized_(false) {
    // 初始化状态向量和协方差矩阵
    x_ = Eigen::VectorXd::Zero(STATE_DIM);
    P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    
    // 初始化测量矩阵 H（将状态映射到观测）
    H_ = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
    // 测量位置 x, y
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    // 测量尺寸 width, length, height
    H_(2, 4) = 1.0;
    H_(3, 5) = 1.0;
    H_(4, 6) = 1.0;
    
    // 初始化状态转移矩阵 F
    F_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    updateTransitionMatrix(dt);
    
    // 初始化过程噪声协方差矩阵 Q
    Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    // 位置过程噪声
    Q_(0, 0) = process_noise_pos * process_noise_pos;
    Q_(1, 1) = process_noise_pos * process_noise_pos;
    // 速度过程噪声
    Q_(2, 2) = process_noise_vel * process_noise_vel;
    Q_(3, 3) = process_noise_vel * process_noise_vel;
    // 尺寸过程噪声 (小一点，因为尺寸一般不会剧烈变化)
    Q_(4, 4) = process_noise_pos * process_noise_pos * 0.1;
    Q_(5, 5) = process_noise_pos * process_noise_pos * 0.1;
    Q_(6, 6) = process_noise_pos * process_noise_pos * 0.1;
    
    // 初始化测量噪声协方差矩阵 R
    R_ = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
    // 位置测量噪声
    R_(0, 0) = measurement_noise_pos * measurement_noise_pos;
    R_(1, 1) = measurement_noise_pos * measurement_noise_pos;
    // 尺寸测量噪声
    R_(2, 2) = measurement_noise_size * measurement_noise_size;
    R_(3, 3) = measurement_noise_size * measurement_noise_size;
    R_(4, 4) = measurement_noise_size * measurement_noise_size;
}

void KalmanFilter::updateTransitionMatrix(double dt) {
    // 更新状态转移矩阵 F
    // 位置 = 位置 + 速度 * dt
    F_(0, 2) = dt;  // x = x + vx * dt
    F_(1, 3) = dt;  // y = y + vy * dt
    // 速度不变
    // 尺寸不变
}

void KalmanFilter::initState(double x, double y, double vx, double vy, 
                            double width, double length, double height) {
    x_ << x, y, vx, vy, width, length, height;
    initialized_ = true;
}

void KalmanFilter::predict(double dt) {
    if (!initialized_) {
        return;
    }
    
    // 如果提供了新的 dt，则更新状态转移矩阵
    if (dt > 0) {
        dt_ = dt;
        updateTransitionMatrix(dt);
    }
    
    // 预测状态向量
    x_ = F_ * x_;
    
    // 预测协方差矩阵
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    if (!initialized_) {
        return;
    }
    
    // 计算卡尔曼增益
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    // 更新状态向量
    x_ = x_ + K * (z - H_ * x_);
    
    // 更新协方差矩阵
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::updatePosition(const Eigen::Vector2d& pos) {
    if (!initialized_) {
        return;
    }
    
    // 创建位置测量向量
    Eigen::VectorXd z = Eigen::VectorXd::Zero(2);
    z << pos(0), pos(1);
    
    // 创建位置测量矩阵
    Eigen::MatrixXd H_pos = Eigen::MatrixXd::Zero(2, STATE_DIM);
    H_pos(0, 0) = 1.0;
    H_pos(1, 1) = 1.0;
    
    // 创建位置测量噪声协方差矩阵
    Eigen::MatrixXd R_pos = Eigen::MatrixXd::Zero(2, 2);
    R_pos(0, 0) = R_(0, 0);
    R_pos(1, 1) = R_(1, 1);
    
    // 计算卡尔曼增益
    Eigen::MatrixXd S = H_pos * P_ * H_pos.transpose() + R_pos;
    Eigen::MatrixXd K = P_ * H_pos.transpose() * S.inverse();
    
    // 更新状态向量
    x_ = x_ + K * (z - H_pos * x_);
    
    // 更新协方差矩阵
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    P_ = (I - K * H_pos) * P_;
}

void KalmanFilter::updateSize(const Eigen::Vector3d& size) {
    if (!initialized_) {
        return;
    }
    
    // 创建尺寸测量向量
    Eigen::VectorXd z = Eigen::VectorXd::Zero(3);
    z << size(0), size(1), size(2);
    
    // 创建尺寸测量矩阵
    Eigen::MatrixXd H_size = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H_size(0, 4) = 1.0;
    H_size(1, 5) = 1.0;
    H_size(2, 6) = 1.0;
    
    // 创建尺寸测量噪声协方差矩阵
    Eigen::MatrixXd R_size = Eigen::MatrixXd::Zero(3, 3);
    R_size(0, 0) = R_(2, 2);
    R_size(1, 1) = R_(3, 3);
    R_size(2, 2) = R_(4, 4);
    
    // 计算卡尔曼增益
    Eigen::MatrixXd S = H_size * P_ * H_size.transpose() + R_size;
    Eigen::MatrixXd K = P_ * H_size.transpose() * S.inverse();
    
    // 更新状态向量
    x_ = x_ + K * (z - H_size * x_);
    
    // 更新协方差矩阵
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    P_ = (I - K * H_size) * P_;
}

Eigen::Vector2d KalmanFilter::getPosition() const {
    Eigen::Vector2d position;
    position << x_(0), x_(1);
    return position;
}

Eigen::Vector2d KalmanFilter::getVelocity() const {
    Eigen::Vector2d velocity;
    velocity << x_(2), x_(3);
    return velocity;
}

Eigen::Vector3d KalmanFilter::getSize() const {
    Eigen::Vector3d size;
    size << x_(4), x_(5), x_(6);
    return size;
}

double KalmanFilter::calculateMahalanobisDistance(const Eigen::VectorXd& z) const {
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    return std::sqrt(y.transpose() * S.inverse() * y);
}

} // namespace obstacles_tracker 