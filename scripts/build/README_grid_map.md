# Grid Map Messages 编译脚本使用说明

## 概述

`grid_map_msgs.sh` 脚本用于编译 grid_map_msgs 包及其依赖项，解决 costmap_generator 等包对 grid_map_msgs 的依赖问题。

## 使用方法

### 1. 确保已克隆 grid_map 源码

如果还没有克隆 grid_map 仓库，请先运行：

```bash
cd src
git clone https://github.com/ANYbotics/grid_map.git
cd ..
```

### 2. 运行编译脚本

```bash
./scripts/build/grid_map_msgs.sh
```

### 3. 加载编译结果

编译完成后，运行以下命令加载新编译的包：

```bash
source install/setup.bash
```

## 脚本功能

该脚本会自动执行以下步骤：

1. **检查源码**: 验证 `src/grid_map` 目录是否存在
2. **切换分支**: 自动切换到 `foxy-devel` 分支（适配 ROS 2 Foxy）
3. **设置环境**: 加载 ROS 2 Foxy 环境和已安装的包
4. **编译依赖**: 先编译 `grid_map_cmake_helpers`
5. **编译主包**: 编译 `grid_map_msgs`

## 编译的包

- `grid_map_cmake_helpers`: CMake 辅助工具
- `grid_map_msgs`: Grid Map 消息定义

## 解决的问题

- 修复 `costmap_generator` 包的 `grid_map_msgs` 依赖问题
- 解决 CMake 找不到 `grid_map_msgsConfig.cmake` 的错误

## 注意事项

1. 确保系统已安装 ROS 2 Foxy
2. 脚本会自动处理分支切换，无需手动操作
3. 编译完成后记得运行 `source install/setup.bash`
4. 如果遇到权限问题，确保脚本有执行权限：`chmod +x scripts/build/grid_map_msgs.sh`

## 故障排除

### 问题：找不到 grid_map 目录
**解决方案**: 运行 `cd src && git clone https://github.com/ANYbotics/grid_map.git`

### 问题：编译失败
**解决方案**: 
1. 检查 ROS 2 环境是否正确设置
2. 确保网络连接正常（可能需要下载依赖）
3. 清理编译缓存：`rm -rf build install log`

### 问题：包覆盖警告
这是正常现象，表示正在覆盖已存在的包，可以安全忽略。
