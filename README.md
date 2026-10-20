# RoboMaster Aim Lab

用于 RoboMaster 自瞄训练的闭环仿真。Godot 负责场景、云台、弹道和命中判定；
C++ ROS 2 桥接向学员提供理想装甲三维位置，并接收瞄准角和开火许可。

```text
Godot 仿真 ←共享内存→ C++ ROS 2 桥接 ←ROS 话题→ 学员自瞄节点
```

学员主要实现目标选择、重力补偿、到位开火，以及进阶的运动预测。

## 环境

| 项目 | 已验证环境 / 要求 |
|---|---|
| 系统 | Ubuntu 24.04，Linux x86_64，同机、同用户运行仿真与桥接 |
| ROS 2 | Jazzy；Humble 尚未实机验证 |
| Godot | 4.7.2，使用仓库脚本下载的固定版本 |
| 编译 | C++17、CMake、colcon |
| 图形 | 有桌面会话和 OpenGL 驱动；纯三维观测可用 headless 模式 |
| Python 3 | 仅用于引擎下载及部分维护工具，ROS 桥接为 C++ |

以下 ROS 步骤以已安装 ROS 2 Jazzy、已配置软件源为前提。
仿真核心也可[不使用 ROS 单独运行](docs/simulator.md)。

## 第一次下载与编译

### 1. 下载源码和引擎

```bash
git clone https://github.com/Supzj9998/rm_aim_sim_training
cd rm_aim_sim
python3 tools/setup_godot.py
./run.sh --headless --editor --import --quit
```

### 2. 安装桥接依赖

```bash
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
```

如果尚未初始化 rosdep，先执行一次 
```bash
sudo rosdep init
```

已经初始化则跳过。
然后在仓库根目录执行：

```bash
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro jazzy -r -y
```

依赖由桥接的 `package.xml` 声明，包括 ROS 标准消息、launch 和 JSON 库。

### 3. 编译 C++ 桥接

```bash
cd ros2_ws
colcon build --packages-select rm_aim_sim_bridge
cd ..
```

`build/`、`install/`、`log/` 会生成在 `ros2_ws/` 内，无需上传或拷贝给其他电脑。
修改桥接源码后重新编译；更新源码后若新增资源，应重新导入 Godot 资源。

## 日常使用

在仓库根目录，一键启动基础考核使用的理想云台场景：

```bash
./run_ros.sh --config=res://configs/ideal.cfg
```

脚本自动加载 ROS 和本工作空间环境、匹配场景通道、启用外部控制。
Ctrl+C 或关闭仿真窗口会同时结束桥接。

其他方式：

```bash
./run_ros.sh                        # 默认场景，云台含伺服响应
./run_ros.sh --headless --config=res://configs/ideal.cfg
./run_ros.sh --config=/path/to/scenario.cfg
```

脚本优先识别当前 ROS 环境，再尝试 Jazzy、Humble；
自定义 ROS 安装位置可设置 `ROS_SETUP=/path/to/setup.bash`。
同通道只启动一套仿真和桥接，不要同时运行 `tools/control_demo.py` 等其他 IPC 客户端。

**只启动仿真和桥接，云台不会自动跟踪或开火。** 还需要在另一个终端启动自己的节点。

## 学员需要写什么

在自己的 ROS 2 工作空间编写 C++ 节点，订阅目标和反馈，计算角度后持续发布控制：

| 方向（学员视角） | 话题 | 消息类型 | 内容 |
|---|---|---|---|
| 接收 | `/aim_sim/armors` | `geometry_msgs/msg/PoseArray` | 可见装甲中心三维位置 |
| 接收 | `/aim_sim/gimbal` | `sensor_msgs/msg/JointState` | 实际 yaw/pitch、角速度 |
| 接收 | `/aim_sim/telemetry` | `diagnostic_msgs/msg/DiagnosticArray` | 弹道参数、统计、血量、通信状态 |
| 接收（建议） | `/aim_sim/session` | `std_msgs/msg/String` | 会话变化时清除旧状态 |
| 自动接收 | `/clock` | `rosgraph_msgs/msg/Clock` | ROS 仿真时钟 |
| 发布 | `/aim_sim/command` | `sensor_msgs/msg/JointState` | 绝对目标 yaw/pitch |
| 发布 | `/aim_sim/enable` | `std_msgs/msg/Bool` | 控制使能 |
| 发布 | `/aim_sim/fire` | `std_msgs/msg/Bool` | 持续开火许可 |

- `odom` 原点在云台旋转中心，X 零位前方、Y 左、Z 上，轴向固定。
- ROS 角度为 rad，yaw 左正、**pitch 抬头为正**；位置为 m。
- `command.header` 使用本次观测的 header；不是系统时间，也不要给旧目标重新填新时间。
- 自己的节点设置标准 ROS 参数 `use_sim_time=true`，不必手写 `/clock` 订阅。
- 建议约 50 Hz 更新控制；默认 250 ms 无新使能或角度指令会停控停火。
- 装甲位置不带稳定 ID，`orientation` 为占位；命中灭灯或战亡时观测会消失。

例如，假设自己的包名为 `student_auto_aim`、已安装可执行文件 `aim_node`，
在自己的工作空间编译后启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run student_auto_aim aim_node --ros-args -p use_sim_time:=true
```

先检查 `ros2 topic echo /aim_sim/armors` 和 `/aim_sim/gimbal`，
再用固定角度、不开火验证控制，最后接入解算和开火逻辑。
字段定义、参数读取和收发片段见 [学员接入说明](docs/student_guide.md)。

## 调整测试场景

| 操作 | 方法 |
|---|---|
| 调整前后位置 | 左侧“初始距离 / m” |
| 调整左右、上下 | “装甲位置 / 整组偏移”，左正、上正，可归零 |
| 取消云台响应 | “理想云台（指令立即到位）” |
| 平移、自转 | 开启对应开关，调整速度、方向、往返半程 |
| 暂停 / 重置 | P / R；重置会恢复血量、清空统计 |
| 切换视角 | V，第三人称可右键拖动、滚轮缩放 |

初始距离是装甲共同中心的前向坐标，不是某块装甲到枪口的精确距离。
目标必须进入相机视场才有观测；出画后是否搜索由学员程序决定。
命中会短暂灭灯，战亡后目标观测持续为空，重置或恢复血量后再继续。

`configs/ideal.cfg` 只开启理想云台，默认目标仍静止。
其他预设为 `translate.cfg`（平移）、`spin.cfg`（自转）、`combined.cfg`（组合）。
使用这些预设时，如需理想云台，在界面中开启或在自己的配置中加入：

```ini
[gimbal]
ideal=true
```

配置格式、范围、保存方法见 [配置参考](docs/configuration.md)。

## 常见问题

| 现象 | 检查 |
|---|---|
| 提示找不到桥接 | 先在 `ros2_ws` 完成 colcon 编译 |
| 仿真启动失败或通道占用 | 关闭同通道的旧仿真、桥接和其他客户端 |
| 学员收不到话题 | 检查 ROS 环境、`ROS_DOMAIN_ID`、节点是否运行及 QoS |
| 云台不动 | 是否有目标；enable 和 command 是否持续更新；header、单位和角度符号是否正确 |
| 有话题但没有装甲 | 是否战亡、灭灯、暂停、关闭外部控制或目标不在视场内 |
| 自动跟随有滞后 | 检查理想云台开关；采样和 ROS 通信仍有离散更新滞后 |
| 转到位但打低 / 动靶打偏 | 检查重力补偿、枪口偏移、弹速及运动提前量 |

## 文档与验证

- [学员接入说明](docs/student_guide.md)：话题字段、时间、坐标与控制要求。
- [ROS 工作空间](ros2_ws/README.md) / [桥接包](ros2_ws/src/rm_aim_sim_bridge/README.md)。
- [仿真详细说明](docs/simulator.md)：模型、血量、弹道、界面、直接 IPC 与相机。
- [完整 IPC 协议](docs/protocol.md)、[直接接入指南](docs/integration.md)、[C++ 图像接入](docs/cpp_integration.md)。
- [配置参考](docs/configuration.md)、[模型说明](docs/models.md)。

```bash
./run.sh --headless --script res://tests/run_tests.gd
python3 tests/integration.py --godot "$PWD/.tools/godot"
```

ROS 测试在 `ros2_ws` 中执行 `colcon test --packages-select rm_aim_sim_bridge`，
再用 `colcon test-result --verbose` 查看结果。
仿真不包含空气阻力、散布、完整遮挡和我方底盘运动；命中率用于算法与接口验证，
不等同于实车命中率。图形模式与 ROS 桥接已在上述本机环境测试，其他环境需自行验证。

原创部分采用 [MIT License](LICENSE)，第三方素材见 [第三方声明](THIRD_PARTY_NOTICES.md)。
本项目非 RoboMaster / DJI 官方产品。
