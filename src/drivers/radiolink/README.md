# radio_link 节点解析说明

radio_link 节点读取 SBUS 遥控器数据，通过串口驱动将其转换为 `bot_msg::msg::RadioLink` 消息，用于车辆线速度、转角、档位与控制模式的高频更新。下文介绍串口帧解析的流程以及关键映射逻辑。

## 数据来源与初始化
- 节点启动时根据参数 `device_name`、`baud_rate` 和 `timeout_ms` 初始化串口，默认配置为 `/dev/ttyUSB0`、100000 波特率。
- `InitValues()` 会将缓存、通道数组与输出消息重置，并初始化计时器，用于周期性发布处理后的 ROS 话题。
- `RadioTimerCallback()` 以 `radio_publish_rate`（默认 50 Hz）将最新解析结果封装为 `RadioLink` 消息发布。

## 字节流收集 `ParseByte`
- 节点在独立线程中循环读取串口数据，每个字节交给 `ParseByte()`。
- 当检测到帧头 `0x0F` 时开始收集，并累计到 25 字节（`kSbusFrameLength`）。
- 若途中再次遇到帧头且当前仅收集了 1 字节，会将帧重头计数以容忍粘包。
- 收集满 25 字节后立即调用 `DecodeFrame()` 处理完整的 SBUS 帧。

## 帧内容拆解 `DecodeFrame`
1. **基础校验**：确认首字节仍为帧头，避免串口噪声误入。
2. **通道值解码**：
   - SBUS 将 16 个通道以 11 bit 对齐编码在 22 个数据字节中。
   - 代码使用位运算从帧中逐一提取通道值，并转换为 `[-1.25, 1.25]` 范围的浮点数。
   - 通过 `FloatLimit()` 将绝对值在 0.02 内的抖动归零。
3. **状态位解析**：
   - 第 24 字节存储 failsafe、frame lost 等状态标志，保存在 `flags_` 中，以便后续扩展。
4. **统计与日志**：解析次数会累积，达到 `log_interval`（默认 25）时输出调试日志，便于监控通道值。
5. **备份原始帧**（可选）：当开启 `enable_info_str_save` 时，将整帧转为十六进制字符串写入指定目录，用于线下诊断。

## 通道到控制量映射 `UpdateMessageFromChannels`
- **速度/转角**：按照参数化的通道索引、线性缩放 `scale` 与偏移 `offset` 将遥控值映射为车辆线速度与转向角。
- **档位**：使用 `gear_channel` 的数值与前进/倒退阈值比较，输出 `1/0/-1`。
- **控制模式**：根据 `mode_channel` 与 `mode_switch_threshold` 判断是否进入遥控模式，结果写入 `control_mode` 位。
- 映射计算均在互斥锁保护下完成，保证与发布计时器之间的数据一致性。

## 关键参数
| 参数名 | 含义 | 默认值 |
| --- | --- | --- |
| `device_name` | 串口设备文件路径 | `/dev/ttyUSB0` |
| `baud_rate` | 串口波特率 | `100000` |
| `radio_publish_rate` | 发布话题频率 (Hz) | `50.0` |
| `linear_channel` / `steering_channel` | 线速度、转角通道索引 | `1` / `0` |
| `gear_channel` / `mode_channel` | 档位、模式通道索引 | `4` / `5` |
| `linear_scale` / `steering_scale` | 通道缩放系数 | `1.0` |
| `linear_offset` / `steering_offset` | 通道偏移 | `0.0` |
| `gear_forward_threshold` / `gear_reverse_threshold` | 档位阈值 | `0.4` / `-0.4` |
| `mode_switch_threshold` | 遥控模式阈值 | `0.4` |
| `enable_debug_log` / `log_interval` | 调试日志开关与间隔 | `false` / `25` |
| `enable_info_str_save` / `info_str_save_dir` | 是否保存原始帧与输出目录 | `false` / `/tmp/radio_logs` |

## 运行建议
- 启动前确认串口已配置为非阻塞并拥有读取权限，必要时使用 `stty -F /dev/ttyUSB0` 检查。
- 调试阶段建议开启 `enable_debug_log`，可快速观察解码后的通道变化。
- 若需要回放遥控输入，启用帧存储功能后收集 `.log` 文件，结合独立分析脚本再现遥控轨迹。
