# rm_aim_sim_bridge

最小 C++17 桥接：读取 Godot 共享内存，发布 ROS 反馈，将 ROS 指令转发回仿真。
编译、运行和测试见 [工作空间说明](../../README.md)。

## 代码

| 文件 | 职责 |
|---|---|
| src/bridge_node.cpp | ROS 发布订阅、坐标和角度转换 |
| src/mailbox.cpp | 共享内存 JSON 收发、客户端锁、指令序号 |
| include/rm_aim_sim_bridge/control_gate.hpp | 超时、暂停与会话变化时清除控制 |
| launch/bridge.launch.xml | 启动节点 |

## 话题

所有话题默认使用可靠传输、depth=1；session 额外持久保留最近值。

| 话题 | ROS 自带类型 | 内容 |
|---|---|---|
| /aim_sim/armors | geometry_msgs/msg/PoseArray | 可见装甲中心三维位置 |
| /aim_sim/gimbal | sensor_msgs/msg/JointState | 实际 yaw/pitch 和角速度 |
| /aim_sim/telemetry | diagnostic_msgs/msg/DiagnosticArray | weapon、stats、health、ipc 具名键值 |
| /aim_sim/session | std_msgs/msg/String | 发布进程与会话标识，变化时丢弃旧目标 |
| /clock | rosgraph_msgs/msg/Clock | 仿真时间 |
| /aim_sim/command | sensor_msgs/msg/JointState | 输入：绝对目标 yaw/pitch |
| /aim_sim/enable | std_msgs/msg/Bool | 输入：外部控制许可 |
| /aim_sim/fire | std_msgs/msg/Bool | 输入：持续开火许可 |

armors 和 gimbal 的 header 来自同次采样，frame_id=odom。
PoseArray 只提供位置，不带稳定装甲 ID；orientation 为单位四元数占位。
装甲背面、四角未完整入画、灯条熄灭时不输出。暂停、关闭外部控制或状态断流时输出空列表。
没有新观测时，学员应停止使用旧目标。

telemetry 的 status.name 为 aim_sim/weapon 等，values 使用 KeyValue 字符串。
weapon 包含 speed、gravity、barrel_length、rate、delay；
stats 包含 fired、hit、resolved、in_flight、hit_rate。
机械限位不在当前 IPC 中，需要查看仿真场景配置。

## 坐标与角度

odom 原点为云台旋转中心，X 零位前方、Y 左、Z 上，轴向固定，距离单位 m。
目标位置 = 装甲世界位置 − 云台世界原点。

ROS 侧 yaw 左正、pitch **抬头正**，单位 rad，均为绝对角。
桥接内部将 pitch 取反，转换成仿真的下正约定。不发布 TF。

## 发送指令

学员节点设置 use_sim_time=true，以约 50 Hz 持续发送 command、enable、fire。
command.header 使用本次观测的 header，name 与 position 一一对应：

```cpp
sensor_msgs::msg::JointState command;
command.header = observation.header;
command.name = {"yaw", "pitch"};
command.position = {target_yaw, target_pitch};
command_pub->publish(command);

std_msgs::msg::Bool enable;
enable.data = true;
enable_pub->publish(enable);

std_msgs::msg::Bool fire;
fire.data = allow_fire;
fire_pub->publish(fire);
```

首次联调使用固定角度、fire=false。支持 yaw/pitch 两种排列，
velocity、effort 当前忽略。时间戳必须来自仿真，不能使用 Unix 墙钟或为旧目标重填新时间。

默认 250 ms 未收到新 enable 或角度指令时停控停火；
只有 fire 超时则继续跟踪、停止新增射击。超时使用单调墙钟，暂停时仍有效。
暂停、重置和会话变化后清空控制，需要重新发送使能和角度。
Bool 没有时间戳或会话字段，不保证延迟布尔消息的严格跨会话隔离。

fire=false 停止新增触发；enable=false 还会取消待出膛事件。
正常退出桥接尽力发送停止指令；桥接异常退出由仿真看门狗停控。

## 启动参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| channel | default | 与仿真共享内存通道一致 |
| poll_hz | 100.0 | 轮询频率，范围 1–1000 Hz；实际状态发布默认约 60 Hz |
| timeout_sec | 0.25 | 墙钟超时，范围 0.02–0.25 s |
| publish_clock | true | 是否发布 /clock |

参数只在启动时设置。launch 支持 channel 和 publish_clock 参数；
直接运行节点时可设置全部参数：

```bash
ros2 run rm_aim_sim_bridge bridge --ros-args -p channel:=default -p timeout_sec:=0.25
```

同一 ROS domain 内只保留一个 /clock 发布源，多仿真时隔离 domain 或关闭多余时钟。
