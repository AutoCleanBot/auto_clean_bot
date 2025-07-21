# 劲旅无人清扫车项目

## 构建方法

！！！基于ubuntu 20.04，ROS2 foxy

### 1.设备驱动构建

#### 1.1 激光雷达

- `inno_sw_ros2`

#### 1.2 GMSL 相机

注意 GMSL直连接口不支持热插拔，所以需要在连接相机之后，重启控制器设备

- `miivii_gmsl_ros`

  ```bash
  sudo apt install ros-foxy-camera-info-manager
  ```
- `gmsl_ros2`
  当处于其他硬件平台是使用该驱动

  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
  国内使用 rosdepc, 可以使用[一键脚本](https://github.com/fishros/install)安装

  ```
  rosdep install --from-paths src --ignore-src -r -y
  ```
## 项目基本坐标系定义

- 定位以及其他均采用的是东北天坐标系:
  - 北向（North）对应 y 轴正方向
  - 东向（East）对应 x 轴正方向
- 角度定义：
  - 0 弧度：指向正北方向
  - π/2 弧度（90度）：指向正东方向
  - π 弧度（180度）：指向正南方向
  - -π/2 弧度（-90度）：指向正西方向

### 坐标系和frame_id对应关系

目前设置三种frame_id : map, base_link, lidar_link

map: 对应的是地图坐标系, 基本坐标系为ENU坐标系, 其中x轴对应east, y轴对应north, z轴对应up, 正北航向为0, 顺时针递增.

base_link: 对应的车身坐标系(也即车辆后轴坐标系, 注意这里不区分rtk/gnss 与车身坐标系的区别, 认为其为一体), （正前方为y轴正方向, 垂直y轴右侧为x轴正方向, 航向由于由IMU确认, 正北为0, 顺时针递增）。

lidar_link: 激光雷达坐标系(注意可能后续有多个激光雷达, 但是所有激光雷达以及感知坐标系, 目前都转换到lidar_link下), 坐标系为正前方为x轴正方向, 垂直x轴左侧为y轴正方向.

## 路径处理工具

项目提供了两个重要的路径处理工具，位于 `path/` 目录下：

### 1. 轨迹处理工具 (process_trajectory.py)

该工具用于对原始轨迹数据进行密化和平滑处理，提高路径质量。

#### 功能特性
- **路径密化**: 将稀疏的轨迹点按指定间距进行插值密化
- **平滑处理**: 使用Savitzky-Golay滤波器对路径进行平滑
- **曲率计算**: 自动计算路径曲率信息
- **可视化**: 生成处理前后的对比图表
- **安全备份**: 自动备份原始文件

#### 使用方法

```bash
# 基本用法
python path/process_trajectory.py input_trajectory.csv

# 处理示例
python path/process_trajectory.py path/local_record_1.csv
```

#### 输入要求
- CSV文件必须包含 `east` 和 `north` 列（东北坐标）
- 可选包含 `yaw` 或 `heading` 列（航向角，度）

#### 输出文件
- 原始文件会被处理后的数据覆盖（原文件自动备份为 `.bak`）
- 生成可视化图片：`{输入文件名}_processed.png`

#### 处理参数
- **密化间距**: 0.1米（默认）
- **平滑窗口**: 101点（默认）
- **多项式阶数**: 3（默认）

### 2. 边界生成工具 (generate_boundaries.py)

该工具根据中心线轨迹生成左右车道边界线。

#### 功能特性
- **智能边界计算**: 基于路径几何特征计算边界点
- **曲率自适应**: 根据路径曲率动态调整边界宽度
- **平滑处理**: 对生成的边界线进行平滑处理
- **可视化**: 生成包含方向箭头的边界线图表

#### 使用方法

```bash
# 基本用法（默认车道宽度1.5米）
python path/generate_boundaries.py input_trajectory.csv

# 指定车道宽度
python path/generate_boundaries.py input_trajectory.csv --width 1.75

# 不生成图表
python path/generate_boundaries.py input_trajectory.csv --width 1.5 --plot false
```

#### 参数说明
- `input_file`: 输入的轨迹CSV文件路径
- `--width`: 车道半宽（从中心线到边界的距离），默认1.5米
- `--plot`: 是否生成可视化图表，默认为true

#### 输入要求
- CSV文件必须包含 `east` 和 `north` 列
- 至少需要3个数据点才能进行可靠的边界生成

#### 输出文件
- `{输入文件名}_left_boundary.csv`: 左边界线数据
- `{输入文件名}_right_boundary.csv`: 右边界线数据
- `{输入文件名}_boundaries_plot.png`: 边界线可视化图表

#### 输出数据格式
生成的边界文件包含以下列：
- `east`: 东坐标
- `north`: 北坐标
- `yaw`: 几何航向角（度）

### 3. 典型工作流程

推荐的路径处理流程：

```bash
# 步骤1: 处理原始轨迹（密化和平滑）
python path/process_trajectory.py raw_trajectory.csv

# 步骤2: 生成车道边界
python path/generate_boundaries.py raw_trajectory.csv --width 1.75

# 结果文件:
# - raw_trajectory.csv (已处理的中心线)
# - raw_trajectory_left_boundary.csv (左边界)
# - raw_trajectory_right_boundary.csv (右边界)
# - raw_trajectory_processed.png (处理对比图)
# - raw_trajectory_boundaries_plot.png (边界可视化图)
```

### 4. 注意事项

- 所有坐标数据应使用东北天坐标系（ENU）
- 处理大型轨迹文件时可能需要较长时间
- 建议在处理前备份重要的原始数据
- 生成的边界线质量依赖于输入轨迹的质量
- 对于高曲率路段，边界宽度会自动调整以避免重叠

