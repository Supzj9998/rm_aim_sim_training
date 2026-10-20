#include "rm_aim_sim_bridge/control_gate.hpp"
#include "rm_aim_sim_bridge/mailbox.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace rm_aim_sim_bridge
{
using Steady = std::chrono::steady_clock;
using JointState = sensor_msgs::msg::JointState;
using Bool = std_msgs::msg::Bool;

double wall_seconds()
{
  return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
}

builtin_interfaces::msg::Time stamp_from_seconds(double seconds)
{
  const auto ns = static_cast<int64_t>(std::llround(seconds * 1e9));
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(ns / 1000000000LL);
  result.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return result;
}

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("bridge", "/aim_sim"),
    mailbox_(declare_parameter<std::string>("channel", "default", startup_only())),
    gate_(declare_parameter<double>("timeout_sec", .25, startup_only()))
  {
    const auto rate = declare_parameter<double>("poll_hz", 100.0, startup_only());
    const auto timeout = get_parameter("timeout_sec").as_double();
    const auto clock_enabled = declare_parameter<bool>("publish_clock", true, startup_only());
    if (!std::isfinite(rate) || rate < 1 || rate > 1000 ||
      !std::isfinite(timeout) || timeout < .02 || timeout > .25)
    {
      throw std::invalid_argument("poll_hz must be [1,1000], timeout_sec must be [0.02,0.25]");
    }
    const auto latest = rclcpp::QoS(1).reliable().durability_volatile();
    armors_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("armors", latest);
    gimbal_pub_ = create_publisher<JointState>("gimbal", latest);
    telemetry_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("telemetry", latest);
    session_pub_ = create_publisher<std_msgs::msg::String>(
      "session", rclcpp::QoS(1).reliable().transient_local());
    if (clock_enabled) {clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", latest);}
    command_sub_ = create_subscription<JointState>("command", latest,
      [this](JointState::ConstSharedPtr msg) {
        const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (!gate_.angles(msg->name, msg->position, stamp, msg->header.frame_id, wall_seconds())) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected stale or invalid angle command");
        }
      });
    enable_sub_ = create_subscription<Bool>("enable", latest,
      [this](Bool::ConstSharedPtr msg) {gate_.enable(msg->data, wall_seconds());});
    fire_sub_ = create_subscription<Bool>("fire", latest,
      [this](Bool::ConstSharedPtr msg) {gate_.fire(msg->data, wall_seconds());});
    // wall timer 不受 use_sim_time、暂停或 /clock 回退影响。
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {poll();});
    RCLCPP_INFO(get_logger(), "Waiting for IPC channel %s; ROS pitch is positive UP",
      get_parameter("channel").as_string().c_str());
  }

private:
  static rcl_interfaces::msg::ParameterDescriptor startup_only()
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return descriptor;
  }

  void publish_unavailable()
  {
    geometry_msgs::msg::PoseArray empty;
    empty.header.frame_id = "odom";
    empty.header.stamp = last_stamp_;
    armors_pub_->publish(empty);
  }

  void poll()
  {
    const double now = wall_seconds();
    try {
      if (const auto state = mailbox_.poll()) {
        gate_.update(*state, now);
        publish_state(*state);
      }
      const bool ready = gate_.fresh(now);
      if (!ready && was_ready_) {publish_unavailable();}
      was_ready_ = ready;
      const auto command = gate_.command(now);
      if (gate_.ready(now) || !ready) {
        mailbox_.send(command.control, command.fire, command.yaw, command.pitch);
      }
    } catch (const std::exception & error) {
      gate_.clear();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
      if (was_ready_) {publish_unavailable(); was_ready_ = false;}
      try {mailbox_.send();} catch (const std::exception &) {}
    }
  }

  void publish_state(const Json & state)
  {
    last_stamp_ = stamp_from_seconds(state.at("sim_time").get<double>());
    if (clock_pub_) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = last_stamp_;
      clock_pub_->publish(clock);
    }
    const Json context = Json::array({state.at("publisher_id"), state.at("session_id")});
    if (context != last_context_) {
      std_msgs::msg::String session;
      session.data = context[0].get<std::string>() + "/" + context[1].get<std::string>();
      session_pub_->publish(session);
      last_context_ = context;
    }
    geometry_msgs::msg::PoseArray armors;
    armors.header.frame_id = "odom";
    armors.header.stamp = last_stamp_;
    const bool paused = state.at("paused").get<bool>();
    const bool enabled = state.at("ipc").at("enabled").get<bool>();
    const auto & actual = state.at("gimbal");
    if (!paused && enabled) {
      for (const auto & armor : state.at("truth").at("armors")) {
        bool visible = false;
        for (const auto & observation : state.at("camera").at("armors")) {
          visible = visible || armor.at("id") == observation.at("id");
        }
        if (!visible) {continue;}
        geometry_msgs::msg::Pose pose;
        // 只平移原点：仿真世界和考核 odom 的轴向一致。
        pose.position.x = armor.at("center")[0].get<double>() - actual.at("origin")[0].get<double>();
        pose.position.y = armor.at("center")[1].get<double>() - actual.at("origin")[1].get<double>();
        pose.position.z = armor.at("center")[2].get<double>() - actual.at("origin")[2].get<double>();
        pose.orientation.w = 1.0;  // 占位，不提供装甲姿态。
        armors.poses.push_back(pose);
      }
    }
    armors_pub_->publish(armors);
    JointState joints;
    joints.header = armors.header;
    joints.name = {"yaw", "pitch"};
    joints.position = {actual.at("yaw").get<double>(), -actual.at("pitch").get<double>()};
    joints.velocity = {actual.at("yaw_vel").get<double>(), -actual.at("pitch_vel").get<double>()};
    gimbal_pub_->publish(joints);
    diagnostic_msgs::msg::DiagnosticArray telemetry;
    telemetry.header = armors.header;
    for (const auto & name : {"weapon", "stats", "health", "ipc"}) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = std::string("aim_sim/") + name;
      status.hardware_id = state.at("publisher_id").get<std::string>();
      status.level = paused || !enabled ?
        diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = paused ? "paused" : (enabled ? "running" : "external control disabled");
      for (const auto & item : state.at(name).items()) {
        diagnostic_msgs::msg::KeyValue value;
        value.key = item.key();
        value.value = item.value().is_string() ? item.value().get<std::string>() : item.value().dump();
        status.values.push_back(value);
      }
      diagnostic_msgs::msg::KeyValue paused_value;
      paused_value.key = "paused";
      paused_value.value = paused ? "true" : "false";
      status.values.push_back(paused_value);
      telemetry.status.push_back(status);
    }
    telemetry_pub_->publish(telemetry);
  }

  Mailbox mailbox_;
  ControlGate gate_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr armors_pub_;
  rclcpp::Publisher<JointState>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr session_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<Bool>::SharedPtr enable_sub_, fire_sub_;
  builtin_interfaces::msg::Time last_stamp_;
  Json last_context_;
  bool was_ready_{false};
};
}  // namespace rm_aim_sim_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<rm_aim_sim_bridge::BridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("aim_sim_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
