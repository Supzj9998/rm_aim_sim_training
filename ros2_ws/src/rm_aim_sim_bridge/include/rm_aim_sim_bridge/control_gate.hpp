#pragma once
#include <cmath>
#include <limits>
#include <string>
#include <vector>
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
    enabled_ = fire_ = angles_valid_ = false;
    enable_at_ = angles_at_ = fire_at_ = -std::numeric_limits<double>::infinity();
  }
  void update(const nlohmann::json & state, double now)
  {
    const auto context = nlohmann::json::array({
      state.at("publisher_id"), state.at("session_id")});
    if (context != context_ || state.at("paused").get<bool>() || !state.at("ipc").at("enabled").get<bool>()) {
      clear();
      minimum_stamp_ = state.at("sim_time").get<double>();
    }
    context_ = context;
    sim_time_ = state.at("sim_time").get<double>();
    active_ = !state.at("paused").get<bool>() && state.at("ipc").at("enabled").get<bool>();
    state_at_ = now;
  }
  bool fresh(double now) const {return now - state_at_ <= timeout_;}
  bool ready(double now) const {return active_ && fresh(now);}
  void enable(bool value, double now)
  {
    if (!value || !ready(now)) {clear(); return;}
    enabled_ = true;
    enable_at_ = now;
  }
  bool angles(const std::vector<std::string> & names, const std::vector<double> & positions,
    double stamp, const std::string & frame, double now)
  {
    if (!ready(now) || frame != "odom" || names.size() != 2 || positions.size() != 2 ||
      !((names[0] == "yaw" && names[1] == "pitch") || (names[0] == "pitch" && names[1] == "yaw")) ||
      !std::isfinite(stamp) || stamp < minimum_stamp_ || sim_time_ - stamp > timeout_ ||
      stamp - sim_time_ > .05 || !std::isfinite(positions[0]) || !std::isfinite(positions[1]) ||
      std::abs(positions[0]) > 1000 || std::abs(positions[1]) > 1000) {
      clear(); return false;
    }
    yaw_ = positions[names[0] == "yaw" ? 0 : 1];
    pitch_ = -positions[names[0] == "pitch" ? 0 : 1];
    angles_valid_ = true;
    angles_at_ = now;
    return true;
  }
  void fire(bool value, double now)
  {
    fire_ = value && ready(now) && enabled_ && now - enable_at_ <= timeout_;
    fire_at_ = now;
  }
  Command command(double now)
  {
    if (!ready(now) || (enabled_ && now - enable_at_ > timeout_) ||
      (angles_valid_ && now - angles_at_ > timeout_)) {clear();}
    if (!enabled_ || !angles_valid_) {return {};}
    return {true, fire_ && now - fire_at_ <= timeout_, yaw_, pitch_};
  }
private:
  double timeout_;
  double state_at_{-std::numeric_limits<double>::infinity()};
  double enable_at_{-std::numeric_limits<double>::infinity()};
  double angles_at_{-std::numeric_limits<double>::infinity()};
  double fire_at_{-std::numeric_limits<double>::infinity()};
  double sim_time_{0}, minimum_stamp_{0}, yaw_{0}, pitch_{0};
  bool active_{false}, enabled_{false}, fire_{false}, angles_valid_{false};
  nlohmann::json context_;
};
}  // namespace rm_aim_sim_bridge
