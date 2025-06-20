#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace obstacles_tracker {

/**
 * @brief 卡尔曼滤波器类，用于对单个障碍物进行状态估计
 */
class KalmanFilter {
public:
    /**
     * @brief 构造函数，初始化卡尔曼滤波器
     * @param dt 时间间隔 (秒)
     * @param process_noise_pos 位置过程噪声标准差
     * @param process_noise_vel 速度过程噪声标准差
     * @param measurement_noise_pos 位置测量噪声标准差
     * @param measurement_noise_size 尺寸测量噪声标准差
     */
    KalmanFilter(double dt = 0.1, 
                 double process_noise_pos = 0.1, 
                 double process_noise_vel = 0.2,
                 double measurement_noise_pos = 0.5, 
                 double measurement_noise_size = 0.2);

    /**
     * @brief 初始化状态向量
     * @param x 初始x位置
     * @param y 初始y位置
     * @param vx 初始x速度
     * @param vy 初始y速度
     * @param width 初始宽度
     * @param length 初始长度
     * @param height 初始高度
     */
    void initState(double x, double y, double vx, double vy, double width, double length, double height);

    /**
     * @brief 预测下一个状态
     * @param dt 时间间隔（如果与初始化时不同）
     */
    void predict(double dt = -1.0);

    /**
     * @brief 更新状态（修正预测）
     * @param z 测量向量 [x, y, width, length, height]
     */
    void update(const Eigen::VectorXd& z);

    /**
     * @brief 更新状态（仅修正位置）
     * @param pos 位置测量 [x, y]
     */
    void updatePosition(const Eigen::Vector2d& pos);

    /**
     * @brief 更新状态（仅修正尺寸）
     * @param size 尺寸测量 [width, length, height]
     */
    void updateSize(const Eigen::Vector3d& size);

    /**
     * @brief 获取当前状态向量
     * @return 状态向量 [x, y, vx, vy, width, length, height]
     */
    const Eigen::VectorXd& getState() const { return x_; }

    /**
     * @brief 获取当前位置
     * @return 位置向量 [x, y]
     */
    Eigen::Vector2d getPosition() const;

    /**
     * @brief 获取当前速度
     * @return 速度向量 [vx, vy]
     */
    Eigen::Vector2d getVelocity() const;

    /**
     * @brief 获取当前尺寸
     * @return 尺寸向量 [width, length, height]
     */
    Eigen::Vector3d getSize() const;
    
    /**
     * @brief 获取当前状态协方差矩阵
     * @return 状态协方差矩阵 P
     */
    const Eigen::MatrixXd& getCovariance() const { return P_; }
    
    /**
     * @brief 计算马氏距离（用于数据关联）
     * @param z 测量向量 [x, y, width, length, height]
     * @return 马氏距离
     */
    double calculateMahalanobisDistance(const Eigen::VectorXd& z) const;
    
    /**
     * @brief 获取时间间隔
     * @return 时间间隔 dt
     */
    double getDt() const { return dt_; }

private:
    // 状态维度：x, y, vx, vy, width, length, height
    static const int STATE_DIM = 7;
    // 测量维度：x, y, width, length, height
    static const int MEAS_DIM = 5;
    // 控制输入维度（这里不使用控制输入）
    static const int CONTROL_DIM = 0;
    
    // 时间间隔
    double dt_;
    
    // 状态向量 [x, y, vx, vy, width, length, height]
    Eigen::VectorXd x_;
    
    // 状态转移矩阵 F
    Eigen::MatrixXd F_;
    
    // 测量矩阵 H
    Eigen::MatrixXd H_;
    
    // 过程噪声协方差矩阵 Q
    Eigen::MatrixXd Q_;
    
    // 测量噪声协方差矩阵 R
    Eigen::MatrixXd R_;
    
    // 状态协方差矩阵 P
    Eigen::MatrixXd P_;
    
    // 是否已初始化
    bool initialized_;
    
    // 更新状态转移矩阵
    void updateTransitionMatrix(double dt);
};

} // namespace obstacles_tracker 