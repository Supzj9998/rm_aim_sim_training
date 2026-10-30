#pragma once
#include <cmath>
#include <limits>
#include <string>
#include <nlohmann/json.hpp>

namespace rm_aim_sim_bridge
{
struct Command
{
  bool control{false}, fire{false};
  double yaw{0}, pitch{0};  // 仿真 pitch 下正。
};

// now 始终使用单调墙钟，暂停仿真时仍执行超时判断。
class ControlGate
{
public:
  explicit ControlGate(double timeout) : timeout_(timeout) {}
  void clear()
  {
    command_ = {};
    command_at_ = -std::numeric_limits<double>::infinity();
    // Keep the ordering watermark until a new session, including after disable/timeout.
  }
  void update(const nlohmann::json & state, double now)
  {
    const std::string session = state.at("publisher_id").get<std::string>() + "/" +
      state.at("session_id").get<std::string>();
    sim_time_ = state.at("sim_time").get<double>();
    if (session != session_) {
      clear();
      minimum_stamp_ = sim_time_;
      last_command_stamp_ = -std::numeric_limits<double>::infinity();
      session_ = session;
    }
    active_ = !state.at("paused").get<bool>() && state.at("ipc").at("enabled").get<bool>();
    if (!active_) {clear(); minimum_stamp_ = sim_time_;}
    state_at_ = now;
  }
  bool fresh(double now) const {return now - state_at_ <= timeout_;}
  bool ready(double now) const {return active_ && fresh(now);}
  bool accept(const std::string & session, double stamp, const std::string & frame,
    double yaw, double pitch, bool enable, bool fire, double now)
  {
    if (!ready(now) || session != session_ || frame != "odom" ||
      !std::isfinite(stamp) || stamp < minimum_stamp_ || stamp < last_command_stamp_ ||
      sim_time_ - stamp > timeout_ || stamp - sim_time_ > .05 ||
      !std::isfinite(yaw) || !std::isfinite(pitch) ||
      std::abs(yaw) > 1000 || std::abs(pitch) > 1000) {
      // Rejected packets neither overwrite a valid decision nor refresh its watchdog.
      return false;
    }
    last_command_stamp_ = stamp;
    command_at_ = now;
    command_ = {enable, enable && fire, yaw, -pitch};
    return true;
  }
  Command command(double now)
  {
    if (!ready(now) || now - command_at_ > timeout_ ||
      sim_time_ - last_command_stamp_ > timeout_) {clear();}
    return command_;
  }
private:
  double timeout_;
  double state_at_{-std::numeric_limits<double>::infinity()};
  double command_at_{-std::numeric_limits<double>::infinity()};
  double last_command_stamp_{-std::numeric_limits<double>::infinity()};
  double sim_time_{0}, minimum_stamp_{0};
  bool active_{false};
  std::string session_;
  Command command_;
};
}  // namespace rm_aim_sim_bridge
