#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace control {

// 车辆状态结构体
struct VehicleState {
    double x;                  // 位置x (东向)
    double y;                  // 位置y (北向)
    double yaw;                // 航向角 (弧度)
    double velocity;           // 速度 (m/s)
    double lateral_error;      // 横向误差 (m)
    double heading_error;      // 航向误差 (弧度)
    double lateral_error_rate; // 横向误差变化率 (m/s)
    double heading_error_rate; // 航向误差变化率 (rad/s)
};

// LQR控制器类
class LQRController {
public:
    // 构造函数
    LQRController();
    
    // 析构函数
    ~LQRController() = default;
    
    // 初始化LQR参数
    void Init(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);
    
    // 更新系统矩阵
    void UpdateSystemMatrix(double velocity);
    
    // 求解Riccati方程
    void SolveRiccatiEquation();
    
    // 计算控制输出
    double ComputeControlCommand(const VehicleState &state);
    
    // 设置车辆参数
    void SetVehicleParams(double wheelbase, double cf, double cr, double mass, double iz);
    
    // 设置求解器参数
    void SetSolverParams(int max_iterations, double tolerance);
    
    // 获取增益矩阵
    const Eigen::MatrixXd& GetGainMatrix() const { return K_; }
    
    // 状态向量和控制向量的维度
    static const int STATE_DIM = 4;  // [e, e_dot, theta_e, theta_e_dot]
    static const int CONTROL_DIM = 1; // 转向角
    
    // 离散化时间步长
    double ts_ = 0.02;  // 控制周期，默认20ms
    
private:
    // 系统矩阵
    Eigen::MatrixXd A_;  // 状态矩阵
    Eigen::MatrixXd B_;  // 控制矩阵
    
    // 离散化系统矩阵
    Eigen::MatrixXd Ad_; // 离散状态矩阵
    Eigen::MatrixXd Bd_; // 离散控制矩阵
    
    // 权重矩阵
    Eigen::MatrixXd Q_;  // 状态权重
    Eigen::MatrixXd R_;  // 控制权重
    
    // 反馈增益矩阵
    Eigen::MatrixXd K_;
    
    // 车辆参数
    double wheelbase_;  // 轴距 (m)
    double cf_;         // 前轮侧偏刚度 (N/rad)
    double cr_;         // 后轮侧偏刚度 (N/rad)
    double mass_;       // 车辆质量 (kg)
    double iz_;         // 车辆转动惯量 (kg*m^2)
    double lf_;         // 前轴到质心距离 (m)
    double lr_;         // 后轴到质心距离 (m)
    
    // 求解器参数
    int max_iterations_; // 最大迭代次数
    double tolerance_;   // 收敛容差
    
    // 将连续系统离散化
    void Discretize();
};

} // namespace control 