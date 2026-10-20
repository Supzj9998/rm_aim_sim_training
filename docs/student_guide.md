# 学员 ROS 2 接入说明

先按 [项目 README](../README.md) 下载、编译并启动仿真与桥接。
本页只说明公开接口，不提供完整自瞄算法。

## 节点职责

学员程序获取可见装甲位置和实际云台姿态，读取弹道参数，自行实现：

1. 目标选择。
2. 三维位置到 yaw/pitch 的解算，包含重力补偿和枪口偏移处理。
3. 到位和可达判断、开火许可。
4. 无目标或数据过期时停控停火。
5. 进阶题中的速度估计、预测与装甲关联。

## 坐标约定

`odom` 原点为云台旋转中心，X 指向零位前方，Y 向左，Z 向上，轴向固定。
装甲位置单位 m，角度单位 rad。ROS yaw 向左为正，pitch **抬头为正**。
命令是绝对目标角，不是相对当前角度的增量。

装甲上下左右场景偏移已包含在消息位置中，学员不需要重复加偏移。
仿真底层 pitch 下正的转换由桥接完成。

## 接收话题

| 话题 | 类型 | 内容 |
|---|---|---|
| /aim_sim/armors | geometry_msgs/msg/PoseArray | 可见装甲中心 |
| /aim_sim/gimbal | sensor_msgs/msg/JointState | 实际角度与角速度 |
| /aim_sim/telemetry | diagnostic_msgs/msg/DiagnosticArray | 弹道参数和状态 |
| /aim_sim/session | std_msgs/msg/String | 会话标识 |

### 装甲位置

```text
header.stamp             观测仿真时间
header.frame_id          "odom"
poses[i].position        [x,y,z]，m
poses[i].orientation     单位四元数占位，不代表真实装甲姿态
```

列表为空表示当前没有有效观测。只输出正面朝向相机、四角完整入画、
灯条点亮的装甲。不提供稳定 ID，不能将数组下标当成同一块装甲。

### 云台反馈

```text
header.stamp       采样仿真时间
header.frame_id    "odom"
name               ["yaw","pitch"]
position           [实际yaw, 实际pitch]，rad
velocity           [yaw角速度, pitch角速度]，rad/s
effort             空
```

与同次采样的 armors 使用相同 header。使用最新反馈判断是否到位。

### 弹道参数与状态

`telemetry.status[]` 用 `name` 区分分组，每组 `values[]` 是 KeyValue，
`value` 为字符串，数字需要自行转换：

| status.name | 键 | 含义 |
|---|---|---|
| aim_sim/weapon | speed | 出膛速度，m/s |
| aim_sim/weapon | gravity | 重力加速度大小，m/s²，方向为 −Z |
| aim_sim/weapon | barrel_length | 云台旋转中心到枪口的距离，m |
| aim_sim/weapon | rate / delay | 射频 Hz / 发射延迟 s |
| aim_sim/weapon | damage | 单发伤害 |
| aim_sim/stats | fired / hit / resolved | 出膛数 / 命中数 / 已结算数 |
| aim_sim/stats | in_flight / pending / hit_rate | 在飞数 / 待出膛数 / 命中率 0–1 |
| aim_sim/health | current / maximum / alive | 当前血量 / 上限 / 存活状态 |
| aim_sim/ipc | enabled / last_seq / received / rejected | 外部模式使能 / 最后指令号 / 收包数 / 拒绝数 |

每组还包含 `paused`。ipc.enabled 指仿真是否打开外部控制模式，
不是学员在 enable 话题上发送的值。机械限位目前从教师提供的场景配置读取。
命中率为 hit/resolved，在飞弹丸不作为未命中；等待弹丸结算后再记录成绩。

### 会话与时间

session.data 为发布进程和仿真会话标识。重置、暂停恢复等操作改变会话时，
清空旧目标、速度估计、滤波器和排队结果。

`use_sim_time` 是 ROS 自带参数。自己的节点应开启它，ROS 会自动接收
`/clock`（rosgraph_msgs/msg/Clock），不用自己编写时钟订阅。
此时 `this->now()` 返回仿真时间；观测间隔用 header.stamp 相减。
暂停时仿真时间停止，重置后可能回退，不能与 Unix 时间混算。
通信超时使用单调墙钟，避免暂停时超时检查也停止。

## 发布话题

| 话题 | 类型 | 内容 |
|---|---|---|
| /aim_sim/command | sensor_msgs/msg/JointState | 绝对目标 yaw/pitch |
| /aim_sim/enable | std_msgs/msg/Bool | 是否允许控制 |
| /aim_sim/fire | std_msgs/msg/Bool | 是否持续允许射击 |

首次联调先发固定角度、fire=false，确认实际姿态与目标一致。
以下为节点内部的发送片段，发布者需自行创建：

```cpp
sensor_msgs::msg::JointState command;
command.header = observation.header;
command.name = {"yaw", "pitch"};
command.position = {target_yaw, target_pitch};
command_pub->publish(command);

std_msgs::msg::Bool enable;
enable.data = have_valid_target;
enable_pub->publish(enable);

std_msgs::msg::Bool fire;
fire.data = allow_fire;
fire_pub->publish(fire);
```

command 的 name 也支持 pitch/yaw 排列，但 position 必须对应；
velocity 和 effort 当前忽略。不能用更改时间戳的方式反复使用旧目标。

话题默认 reliable、volatile、depth=1；session 为 transient_local。
建议约 50 Hz 更新三个控制话题。默认 250 ms 角度或使能接收超时会停控停火，
只有 fire 超时则继续跟踪、停止新增射击。

fire=true 按仿真射频持续触发，不是一条消息打一发；
fire=false 停止新增触发，已接受的待出膛仍会完成。
enable=false 会同时取消待出膛事件，已出膛弹丸继续飞行。

## 查看接口

```bash
ros2 topic list
ros2 topic echo /aim_sim/armors
ros2 topic echo /aim_sim/gimbal
ros2 topic echo /aim_sim/telemetry
ros2 interface show sensor_msgs/msg/JointState
```

自己的节点与桥接使用同一个 ROS_DOMAIN_ID。
话题属于 ROS 标准消息，无需依赖桥接包头文件或 Godot SDK。
C++ 通常只需依赖 rclcpp、geometry_msgs、sensor_msgs、std_msgs、diagnostic_msgs。
