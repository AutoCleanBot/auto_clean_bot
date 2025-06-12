#include "control/lqr_controller.h"
#include <iostream>

namespace control {

// 定义静态常量
const int LQRController::STATE_DIM;
const int LQRController::CONTROL_DIM;

LQRController::LQRController() 
    : wheelbase_(0.0), cf_(0.0), cr_(0.0), mass_(0.0), iz_(0.0),
      lf_(0.0), lr_(0.0), max_iterations_(100), tolerance_(0.01) {
    // 初始化矩阵
    A_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    B_ = Eigen::MatrixXd::Zero(STATE_DIM, CONTROL_DIM);
    Ad_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    Bd_ = Eigen::MatrixXd::Zero(STATE_DIM, CONTROL_DIM);
    Q_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    R_ = Eigen::MatrixXd::Identity(CONTROL_DIM, CONTROL_DIM);
    K_ = Eigen::MatrixXd::Zero(CONTROL_DIM, STATE_DIM);
}

void LQRController::Init(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R) {
    Q_ = Q;
    R_ = R;
}

void LQRController::SetVehicleParams(double wheelbase, double cf, double cr, double mass, double iz) {
    wheelbase_ = wheelbase;
    cf_ = cf;
    cr_ = cr;
    mass_ = mass;
    iz_ = iz;
    
    // 计算前后轴到质心的距离
    // 假设质心在轴距的中点，可以根据实际情况调整
    lf_ = wheelbase_ * 0.5;
    lr_ = wheelbase_ * 0.5;
}

void LQRController::SetSolverParams(int max_iterations, double tolerance) {
    max_iterations_ = max_iterations;
    tolerance_ = tolerance;
}

void LQRController::UpdateSystemMatrix(double velocity) {
    // 确保速度不为零，避免除零错误
    double v = std::max(velocity, 0.1);
    
    // 基于自行车模型构建连续时间系统矩阵
    // 状态向量: [e, e_dot, theta_e, theta_e_dot]
    // e: 横向误差
    // e_dot: 横向误差变化率
    // theta_e: 航向误差
    // theta_e_dot: 航向误差变化率
    
    // 状态矩阵A
    A_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    A_(0, 1) = 1.0;
    A_(1, 1) = -(cf_ + cr_) / (mass_ * v);
    A_(1, 2) = (cf_ + cr_) / mass_;
    A_(1, 3) = (lf_ * cf_ - lr_ * cr_) / (mass_ * v);
    A_(2, 3) = 1.0;
    A_(3, 1) = (lf_ * cf_ - lr_ * cr_) / (iz_ * v);
    A_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    A_(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (iz_ * v);
    
    // 控制矩阵B
    B_ = Eigen::MatrixXd::Zero(STATE_DIM, CONTROL_DIM);
    B_(1, 0) = cf_ / mass_;
    B_(3, 0) = lf_ * cf_ / iz_;
    
    // 离散化系统
    Discretize();
}

void LQRController::Discretize() {
    // 使用一阶近似离散化连续时间系统
    // Ad = I + A * ts
    // Bd = B * ts
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    Ad_ = I + A_ * ts_;
    Bd_ = B_ * ts_;
}

void LQRController::SolveRiccatiEquation() {
    // 求解离散时间代数Riccati方程
    // 使用迭代法求解
    
    // 初始化P矩阵
    Eigen::MatrixXd P = Q_;
    Eigen::MatrixXd P_next;
    
    // 迭代求解
    for (int i = 0; i < max_iterations_; ++i) {
        // 计算P_next
        Eigen::MatrixXd temp = R_ + Bd_.transpose() * P * Bd_;
        Eigen::MatrixXd K_temp = temp.inverse() * Bd_.transpose() * P * Ad_;
        P_next = Ad_.transpose() * P * Ad_ - 
                 Ad_.transpose() * P * Bd_ * K_temp + Q_;
        
        // 检查收敛性
        if ((P_next - P).norm() < tolerance_) {
            break;
        }
        
        P = P_next;
    }
    
    // 计算反馈增益矩阵K
    Eigen::MatrixXd temp = R_ + Bd_.transpose() * P * Bd_;
    K_ = temp.inverse() * Bd_.transpose() * P * Ad_;
}

double LQRController::ComputeControlCommand(const VehicleState &state) {
    // 构建状态向量
    Eigen::VectorXd x(STATE_DIM);
    x << state.lateral_error, state.lateral_error_rate, 
         state.heading_error, state.heading_error_rate;
    
    // 计算控制输出
    Eigen::VectorXd u = -K_ * x;
    
    // 返回转向控制命令
    return u(0);
}

} // namespace control 