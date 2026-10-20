# ROS 2 工作空间

本工作空间只包含 C++ 仿真桥接，消息全部使用 ROS 自带类型。

```text
ros2_ws/
├── src/
│   └── rm_aim_sim_bridge/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/rm_aim_sim_bridge/
│       ├── src/
│       ├── launch/
│       └── test/
├── build/      # colcon 生成，不提交
├── install/    # colcon 生成，不提交
└── log/        # colcon 生成，不提交
```

## 编译

首次下载、引擎导入和依赖安装请先按 [项目 README](../README.md) 完成。
在仓库根目录可使用 rosdep 根据本工作空间的 package.xml 安装依赖。

从仓库根目录进入工作空间：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select rm_aim_sim_bridge
source install/setup.bash
```

依赖：rclcpp、geometry_msgs、sensor_msgs、std_msgs、diagnostic_msgs、
rosgraph_msgs、nlohmann_json 和标准 ROS launch。
本机已验证 Jazzy；Humble 用户替换环境路径，尚未在 Humble 实测。

## 运行

编译完成后，最简单的方式是在仿真仓库根目录执行：

```bash
./run_ros.sh
# 可选参数：--headless、--config=res://configs/ideal.cfg
```

脚本自动加载 ROS 和工作空间环境，匹配配置中的 channel；Ctrl+C 同时结束两个进程。
可设置 `ROS_SETUP=/其他位置/setup.bash` 指定 ROS 安装。
以下是分别手动启动的方式。

终端一，在仓库根目录启动仿真：

```bash
./run.sh -- --remote
# 只需要状态、不看画面时：
# ./run.sh --headless -- --remote
```

终端二，在 ros2_ws 中启动桥接：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rm_aim_sim_bridge bridge.launch.xml
```

同一个共享内存通道只运行一个桥接，不要同时运行 tools 中的控制或相机客户端。
桥接可以早于仿真启动。接口见 [桥接包说明](src/rm_aim_sim_bridge/README.md)。

## 测试

在 ros2_ws 中：

```bash
colcon test --packages-select rm_aim_sim_bridge
colcon test-result --verbose
```

包含 C++ 控制生命周期测试和真实 Godot＋ROS 闭环测试。
集成测试使用独立通道，优先使用 GODOT_BIN，否则使用仓库 .tools/godot；
没有引擎时跳过集成测试。
