# Remote Controller Simulator

这是一个用于测试的遥控器模拟器，可以模拟真实的remote_controller节点发布按键消息。

## 功能说明

模拟器通过键盘输入来发送与真实遥控器相同的按键命令：

- **按键1**: 起步
- **按键2**: 停止  
- **按键3**: 自动驾驶
- **按键4**: 手动接管
- **按键5**: 任务路径切换
- **按键6**: 扩展功能6
- **按键7**: 扩展功能7
- **按键8**: 扩展功能8
- **按键q**: 退出程序

## 使用方法

### 1. 编译
```bash
cd ~/auto_clean_bot
colcon build --packages-select remote_controller_simulator
source install/setup.bash
```

### 2. 运行模拟器
```bash
# 方法1: 直接运行节点
ros2 run remote_controller_simulator remote_controller_simulator_node

# 方法2: 使用launch文件
ros2 launch remote_controller_simulator remote_controller_simulator.launch.py
```

### 3. 测试消息发布
在另一个终端中监听消息：
```bash
ros2 topic echo /remote_controller/cmd
```

### 4. 操作说明
- 启动模拟器后，会显示按键功能说明
- 输入数字1-8然后按回车发送对应命令
- 输入q然后按回车退出程序
- 每次输入都会在终端显示发送的命令信息和确认

## 消息格式

发布的消息格式与真实remote_controller完全一致：
- **话题**: `/remote_controller/cmd`
- **消息类型**: `std_msgs/msg/Int32`
- **数据内容**: 按键编号(1-8)

## 注意事项

1. 确保没有同时运行真实的remote_controller节点，避免话题冲突
2. 模拟器使用非缓冲键盘输入，按键会立即响应
3. 程序退出时会自动恢复终端设置
4. 如果程序异常退出导致终端输入异常，可以运行`reset`命令恢复

## 测试场景

这个模拟器特别适用于以下测试场景：
- 控制模块的按键响应测试
- 自动驾驶状态切换测试  
- 手动/自动模式切换测试
- 路径切换功能测试
- 集成测试中的人工干预模拟