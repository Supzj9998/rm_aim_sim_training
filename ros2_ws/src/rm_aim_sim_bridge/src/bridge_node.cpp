#include "rm_aim_sim_bridge/control_gate.hpp"
#include "rm_aim_sim_bridge/mailbox.hpp"
#include "rm_aim_sim_bridge/msg/aim_command.hpp"
#include "rm_aim_sim_bridge/msg/armor_corners_array.hpp"
#include "rm_aim_sim_bridge/msg/gimbal_state.hpp"
#include "rm_aim_sim_bridge/msg/telemetry.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace rm_aim_sim_bridge
{
using Steady = std::chrono::steady_clock;
using AimCommand = rm_aim_sim_bridge::msg::AimCommand;
using ArmorCornersArray = rm_aim_sim_bridge::msg::ArmorCornersArray;
using GimbalState = rm_aim_sim_bridge::msg::GimbalState;
using Telemetry = rm_aim_sim_bridge::msg::Telemetry;

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
    armors_pub_ = create_publisher<ArmorCornersArray>("armors", latest);
    gimbal_pub_ = create_publisher<GimbalState>("gimbal", latest);
    telemetry_pub_ = create_publisher<Telemetry>("telemetry", latest);
    if (clock_enabled) {clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", latest);}
    command_sub_ = create_subscription<AimCommand>("command", latest,
      [this](AimCommand::ConstSharedPtr msg) {
        const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (!gate_.accept(msg->session_id, stamp, msg->header.frame_id,
          msg->yaw, msg->pitch, msg->enable, msg->fire, wall_seconds())) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected stale, out-of-order or invalid command/session");
        }
      });
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
    ArmorCornersArray empty;
    empty.header.frame_id = "odom";
    empty.header.stamp = last_stamp_;
    empty.session_id = last_session_;
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
    last_session_ = state.at("publisher_id").get<std::string>() + "/" +
      state.at("session_id").get<std::string>();
    ArmorCornersArray armors;
    armors.header.frame_id = "odom";
    armors.header.stamp = last_stamp_;
    armors.session_id = last_session_;
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
        rm_aim_sim_bridge::msg::ArmorCorners plate;
        const auto & corners = armor.at("corners");
        // 只平移原点：仿真世界和考核 odom 的轴向一致。
        for (size_t i = 0; i < plate.corners.size(); ++i) {
          plate.corners[i].x = corners.at(i).at(0).get<double>() - actual.at("origin")[0].get<double>();
          plate.corners[i].y = corners.at(i).at(1).get<double>() - actual.at("origin")[1].get<double>();
          plate.corners[i].z = corners.at(i).at(2).get<double>() - actual.at("origin")[2].get<double>();
        }
        armors.armors.push_back(plate);
      }
    }
    armors_pub_->publish(armors);
    GimbalState gimbal;
    gimbal.header = armors.header;
    gimbal.yaw = actual.at("yaw").get<double>();
    gimbal.pitch = -actual.at("pitch").get<double>();
    gimbal.yaw_velocity = actual.at("yaw_vel").get<double>();
    gimbal.pitch_velocity = -actual.at("pitch_vel").get<double>();
    gimbal_pub_->publish(gimbal);
    Telemetry telemetry;
    telemetry.header = armors.header;
    const auto & weapon = state.at("weapon");
    telemetry.speed = weapon.at("speed").get<double>();
    telemetry.gravity = weapon.at("gravity").get<double>();
    telemetry.barrel_length = weapon.at("barrel_length").get<double>();
    telemetry.rate = weapon.at("rate").get<double>();
    telemetry.delay = weapon.at("delay").get<double>();
    telemetry_pub_->publish(telemetry);
  }

  Mailbox mailbox_;
  ControlGate gate_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ArmorCornersArray>::SharedPtr armors_pub_;
  rclcpp::Publisher<GimbalState>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<Telemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<AimCommand>::SharedPtr command_sub_;
  builtin_interfaces::msg::Time last_stamp_;
  std::string last_session_;
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
