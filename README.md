# RoboMaster Aim Lab

用于 RoboMaster 自瞄训练的闭环仿真。Godot 负责场景、云台、弹道和命中判定；
C++ ROS 2 桥接向学员提供可见装甲的四个三维角点，并接收瞄准角和开火许可。

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
| 开发工具 | Git、C++17、CMake、colcon |
| 图形 | 有桌面会话和 OpenGL 驱动；纯三维观测可用 headless 模式 |
| Python 3 | 仅用于引擎下载及部分维护工具，ROS 桥接为 C++ |

以下 ROS 步骤以已安装 ROS 2 Jazzy、已配置软件源为前提。
仅运行仿真可使用 `./run.sh --sim-only`。

## 第一次下载与编译

### 1. 下载源码和引擎

```bash
git clone https://github.com/Supzj9998/rm_aim_sim_training
cd rm_aim_sim_training
python3 tools/setup_godot.py
./run.sh --sim-only --headless --editor --import --quit
```

### 2. 安装桥接依赖

```bash
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
```

如果尚未初始化 rosdep，先执行一次：

```bash
sudo rosdep init
```

已经初始化则跳过。然后在仓库根目录执行：

```bash
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro jazzy -r -y
```

依赖由桥接的 `package.xml` 声明，包括 ROS 标准消息、自定义消息生成工具、launch 和 JSON 库。

### 3. 编译 C++ 桥接

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select rm_aim_sim_bridge
cd ..
```

`build/`、`install/`、`log/` 会生成在 `ros2_ws/` 内，无需上传或拷贝给其他电脑。
修改桥接源码或 `.msg` 后重新编译；修改 `.msg` 后，使用这些消息的学员包也要重新编译。
更新源码后若新增资源，应重新导入 Godot 资源。

## 日常使用

在仓库根目录，一键启动基础考核使用的理想云台场景：

```bash
./run.sh
```

脚本自动加载 ROS 和本工作空间环境、匹配场景通道、启用外部控制。
Ctrl+C 或关闭仿真窗口会同时结束桥接。

其他方式：

```bash
./run.sh --headless
./run.sh --config=res://configs/combined.cfg
./run.sh --config=/path/to/scenario.cfg
```

无界面模式仍提供 ROS 三维观测和控制接口，但不渲染相机图像。

只运行仿真、导入资源或执行 Godot 测试时，将 `--sim-only` 放在第一个参数，例如 `./run.sh --sim-only`；该模式不启动 ROS 桥接。

脚本优先识别当前 ROS 环境，再尝试 Jazzy、Humble；
自定义 ROS 安装位置可设置 `ROS_SETUP=/path/to/setup.bash`。
同通道只启动一套仿真和桥接，不要同时连接其他 IPC 控制客户端。

**只启动仿真和桥接，云台不会自动跟踪或开火。** 还需要在另一个终端启动自己的节点。

## 接入学员节点

学员接收 3 个话题，发送 1 个话题；消息类型都属于 `rm_aim_sim_bridge/msg/`：

| 方向 | 话题 | 消息类型 | 内容 |
|---|---|---|---|
| 接收 | `/aim_sim/armors` | `ArmorCornersArray` | 可见装甲的四个三维角点，odom 坐标 |
| 接收 | `/aim_sim/gimbal` | `GimbalState` | 实际 yaw、pitch 和对应角速度 |
| 接收 | `/aim_sim/telemetry` | `Telemetry` | 弹速、重力、枪管长度、射击频率、发射延迟 |
| 发送 | `/aim_sim/command` | `AimCommand` | 目标 yaw、pitch、控制使能和开火许可 |

完整字段、坐标约定、会话、评分和 C++ 发送示例见 [学员接入说明](docs/student_guide.md)。

在新终端进入仿真仓库根目录，先加载仿真消息接口，再编译、运行自己的包。学员包需要依赖 `rm_aim_sim_bridge`；下面的工作空间路径、包名和可执行文件名需替换为自己工程中的名称：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cd /absolute/path/to/student_ws
colcon build --packages-select student_auto_aim
source install/setup.bash
ros2 run student_auto_aim aim_node --ros-args -p use_sim_time:=true
```

`use_sim_time=true` 让节点自动使用仿真发布的 `/clock`。先用固定角度且不开火验证控制，再接入自己的解算与开火逻辑。

## 调整测试场景

| 操作 | 方法 |
|---|---|
| 调整前后位置 | 左侧“初始距离 / m” |
| 调整左右、上下 | “装甲位置 / 整组偏移”，左正、上正，可归零 |
| 云台响应 | “理想云台（指令立即到位）”默认开启，关闭后使用伺服响应 |
| 平移、自转 | 直接使用“开启平移”“开启自转”开关；展开各自的参数框调整参数 |
| 暂停 / 重置 | P / R；重置会恢复血量、清空统计 |
| 切换视角 | V，第三人称可右键拖动、滚轮缩放 |
| 坐标与角点 | 在“04 场景操作”开启“显示坐标与角点”；O odom、G 云台、C 相机、M 枪口可同时勾选 |
| 射击频率 | 默认 10.0 Hz，即持续开火时每秒 10 发 |

初始距离是装甲共同中心的前向坐标，不是某块装甲到枪口的精确距离。
目标必须进入相机视场才有观测；出画后是否搜索由学员程序决定。
命中会短暂灭灯，战亡后目标观测持续为空，重置或恢复血量后再继续。

默认使用理想云台，目标静止；“平移参数”和“自转参数”分别默认收起，点击对应按钮展开调整。
运动开关位于各自折叠框外，收起参数不会关闭运动。

坐标显示默认关闭，开启后默认只勾选 odom；全部取消勾选时仍显示装甲四角。四角数值始终使用 odom，单位 m。世界坐标系仅用于内部计算，不提供显示选项。

运动预设为 `translate.cfg`（平移）、`spin.cfg`（自转）、`combined.cfg`（组合），同样默认使用理想云台。
`configs/ideal.cfg` 保留为显式启用理想云台的预设。需要伺服响应时，关闭界面的理想云台开关，或在配置文件的 `[gimbal]` 段设置 `ideal=false`。

装甲下沿、伸展、高度差、命中区宽高、外观等设置已从界面隐藏，仍可在场景配置文件中设置。配置使用 Godot ConfigFile 格式，可参考 `configs/default.cfg`；省略字段继承默认值。“保存当前配置”后界面会显示文件位置，下次通过 `--config=路径` 加载。

## 常见问题

| 现象 | 检查 |
|---|---|
| 提示找不到桥接 | 先在 `ros2_ws` 完成 colcon 编译 |
| 仿真启动失败或通道占用 | 关闭同通道的旧仿真、桥接和其他客户端 |
| 学员收不到话题 | 检查 ROS 环境、`ROS_DOMAIN_ID`、节点是否运行及 QoS |
| 云台不动 | 是否有目标；command 是否持续更新且 enable=true；session_id、header、单位和角度符号是否正确 |
| 有话题但没有装甲 | 是否战亡、灭灯、暂停、关闭外部控制或目标不在视场内 |
| 自动跟随有滞后 | 检查理想云台开关；采样和 ROS 通信仍有离散更新滞后 |
| 转到位但打低 / 动靶打偏 | 检查重力补偿、枪口偏移、弹速及运动提前量 |

## 验证

从仓库根目录运行仿真核心和共享内存闭环检查：

```bash
./run.sh --sim-only --headless --script res://tests/run_tests.gd
python3 tests/integration.py --godot "$PWD/.tools/godot"
```

ROS 控制与真实 Godot 闭环检查：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
colcon test --packages-select rm_aim_sim_bridge
colcon test-result --verbose
cd ..
```

仿真不包含空气阻力、散布、完整遮挡和我方底盘运动；命中率用于算法与接口验证，
不等同于实车命中率。图形模式与 ROS 桥接已在上述本机环境测试，其他环境需自行验证。

原创部分采用 [MIT License](LICENSE)。装甲贴图来自 [LabelMaster](https://github.com/tff345/label-robomaster)，对应的 [MIT 许可证](assets/third_party/label_robomaster/LICENSE)和[来源记录](assets/stickers/sources.json)随素材保留。
本项目非 RoboMaster / DJI 官方产品。
