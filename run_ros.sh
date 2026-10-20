#!/usr/bin/env bash
# 一次启动 Godot 仿真和 C++ ROS 桥接，Ctrl+C 同时退出。
set -eo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_config="res://configs/default.cfg"
sim_args=()

while (($#)); do
  case "$1" in
    --headless) sim_args+=(--headless); shift ;;
    --config=*) sim_config="${1#--config=}"; shift ;;
    --config)
      [[ $# -ge 2 ]] || { echo "--config 需要配置文件路径" >&2; exit 2; }
      sim_config="$2"; shift 2 ;;
    -h|--help)
      echo "用法：$0 [--headless] [--config=配置路径]"
      echo "示例：$0 --config=res://configs/ideal.cfg"
      exit 0 ;;
    *) echo "未知参数：$1（使用 --help 查看用法）" >&2; exit 2 ;;
  esac
done

# 支持 Godot res:// 路径和普通文件路径。
case "$sim_config" in
  res://*) sim_config="$SIM_ROOT/${sim_config#res://}" ;;
  /*) ;;
  *) sim_config="$PWD/$sim_config" ;;
esac
[[ -f "$sim_config" ]] || { echo "配置不存在：$sim_config" >&2; exit 1; }

# 未指定 [ipc].channel 时，仿真默认使用 default。
sim_channel=default
sim_section=""
sim_channel_pattern='^[[:space:]]*channel[[:space:]]*=[[:space:]]*"([A-Za-z0-9_-]{1,80})"[[:space:]]*(;.*)?$'
while IFS= read -r sim_line || [[ -n "$sim_line" ]]; do
  if [[ "$sim_line" =~ ^[[:space:]]*\[([^]]+)\] ]]; then
    sim_section="${BASH_REMATCH[1]}"
  elif [[ "$sim_section" == ipc && "$sim_line" =~ $sim_channel_pattern ]]; then
    sim_channel="${BASH_REMATCH[1]}"
  fi
done < "$sim_config"

# 可使用 ROS_SETUP=/其他位置/setup.bash 指定 ROS 安装。
if [[ -z "${ROS_SETUP:-}" ]]; then
  for sim_distro in "${ROS_DISTRO:-}" jazzy humble; do
    if [[ -n "$sim_distro" && -f "/opt/ros/$sim_distro/setup.bash" ]]; then
      ROS_SETUP="/opt/ros/$sim_distro/setup.bash"
      break
    fi
  done
fi
[[ -f "${ROS_SETUP:-}" ]] || { echo "找不到 ROS 环境，请设置 ROS_SETUP" >&2; exit 1; }
source "$ROS_SETUP"
sim_overlay="$SIM_ROOT/ros2_ws/install/local_setup.bash"
[[ -f "$sim_overlay" ]] || {
  echo "请先编译桥接：cd $SIM_ROOT/ros2_ws && colcon build" >&2
  exit 1
}
source "$sim_overlay"
sim_prefix="$(ros2 pkg prefix rm_aim_sim_bridge)"
sim_bridge="$sim_prefix/lib/rm_aim_sim_bridge/bridge"
[[ -x "$sim_bridge" ]] || { echo "桥接可执行文件不存在，请重新 colcon build" >&2; exit 1; }

sim_pid=""
sim_bridge_pid=""
cleanup() {
  trap - EXIT
  for sim_child in "$sim_bridge_pid" "$sim_pid"; do
    [[ -z "$sim_child" ]] || kill -TERM "$sim_child" 2>/dev/null || true
  done
  # 给 ROS/Godot 时间释放锁，避免残留后台进程。
  for ((sim_attempt=0; sim_attempt<30; sim_attempt++)); do
    sim_alive=false
    for sim_child in "$sim_bridge_pid" "$sim_pid"; do
      if [[ -n "$sim_child" ]] && kill -0 "$sim_child" 2>/dev/null; then
        sim_alive=true
      fi
    done
    [[ "$sim_alive" == true ]] || break
    sleep 0.1
  done
  for sim_child in "$sim_bridge_pid" "$sim_pid"; do
    [[ -z "$sim_child" ]] || kill -KILL "$sim_child" 2>/dev/null || true
    [[ -z "$sim_child" ]] || wait "$sim_child" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$SIM_ROOT/run.sh" "${sim_args[@]}" -- --remote "--config=$sim_config" &
sim_pid=$!

# 只接入本次启动的仿真，不能误连同通道的旧实例。
sim_state="/dev/shm/rm_aim_sim_$sim_channel/state.json"
sim_publisher_pattern='"publisher_id"[[:space:]]*:[[:space:]]*"'"$sim_pid"'-'
sim_ready=false
for ((sim_attempt=0; sim_attempt<150; sim_attempt++)); do
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    wait "$sim_pid" || exit $?
    echo "仿真在就绪前退出" >&2
    exit 1
  fi
  if [[ -r "$sim_state" ]]; then
    sim_json="$(cat "$sim_state" 2>/dev/null)" || sim_json=""
    if [[ "$sim_json" =~ $sim_publisher_pattern ]]; then
      sim_ready=true
      break
    fi
  fi
  sleep 0.1
done
[[ "$sim_ready" == true ]] || { echo "等待仿真就绪超时" >&2; exit 1; }

"$sim_bridge" --ros-args -p "channel:=$sim_channel" &
sim_bridge_pid=$!
echo "仿真和 ROS 桥接已启动（channel=$sim_channel），Ctrl+C 同时退出。"
# 任一程序退出，清理另一程序；保留出错程序的退出码。
wait -n "$sim_pid" "$sim_bridge_pid"
