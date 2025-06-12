# 控制节点使用说明

## 概述

本控制节点实现了多种横向控制算法，包括：
1. 混合控制器（Pure Pursuit + Stanley）
2. LQR（线性二次型调节器）控制器

## 参数配置

### 基本参数
- `publish_rate`: 控制命令发布频率
- `preview_time`: 预瞄时间
- `tolerance_distance`: 预瞄距离容差
- `max_steering_angle`: 最大转向角度
- `wheelbase`: 轴距
- `max_linear_velocity`: 最大线速度
- `min_linear_velocity`: 最小线速度
- `acceleration_limit`: 加速度限制
- `deceleration_limit`: 减速度限制

### 混合控制器参数
- `pursuit_control_rate`: 纯追踪控制比例
- `stanley_control_rate`: Stanley控制比例
- `sta_lat_rate`: Stanley横向误差系数
- `feedforward_rate`: 前馈控制比例
- `heading_error_rate`: 航向误差比例

### LQR控制器参数
- `use_lqr_controller`: 是否使用LQR控制器（默认为false）
- `cf`: 前轮侧偏刚度
- `cr`: 后轮侧偏刚度
- `mass`: 车辆质量
- `iz`: 车辆转动惯量
- `lqr_max_iterations`: LQR最大迭代次数
- `lqr_eps`: LQR收敛容差

## 使用LQR控制器

要启用LQR控制器，需要在启动文件中设置参数`use_lqr_controller`为`true`，例如：

```xml
<node pkg="control" exec="control_node" name="control_node">
  <param name="use_lqr_controller" value="true"/>
  <param name="cf" value="155494.663"/>
  <param name="cr" value="155494.663"/>
  <param name="mass" value="1500.0"/>
  <param name="iz" value="2500.0"/>
</node>
```

## 调优建议

### LQR控制器调优

LQR控制器的性能主要取决于状态权重矩阵Q和控制权重矩阵R。在代码中，这些矩阵被初始化为：

```cpp
Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
Q(0, 0) = 25.0;  // 横向误差权重
Q(1, 1) = 3.0;   // 横向误差变化率权重
Q(2, 2) = 20.0;  // 航向误差权重
Q(3, 3) = 1.0;   // 航向误差变化率权重

Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
R(0, 0) = 20.0;  // 控制输入权重
```

调整这些权重可以改变控制器的响应特性：
- 增大Q中的权重会使控制器更积极地减小相应的状态误差
- 增大R中的权重会使控制器更保守，减小控制输入的幅度

## 注意事项

1. 使用LQR控制器需要准确的车辆动力学参数，如侧偏刚度、质量和转动惯量
2. LQR控制器在低速情况下可能表现不佳，因为线性模型在低速时不够准确
3. 前馈控制项对于提高弯道跟踪性能非常重要
