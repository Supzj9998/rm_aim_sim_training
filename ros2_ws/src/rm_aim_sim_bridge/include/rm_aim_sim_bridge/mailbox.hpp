#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <nlohmann/json.hpp>

namespace rm_aim_sim_bridge
{
using Json = nlohmann::json;

// 单个桥接持有客户端锁；poll 不阻塞等待下一帧。
class Mailbox
{
public:
  explicit Mailbox(const std::string & channel);
  ~Mailbox();
  Mailbox(const Mailbox &) = delete;
  Mailbox & operator=(const Mailbox &) = delete;
  std::optional<Json> poll();
  void send(bool control = false, bool fire = false, double yaw = 0, double pitch = 0);
  bool connected() const {return !state_.is_null();}
private:
  std::string path_;
  int directory_{-1}, lock_{-1};
  Json state_, identity_, context_;
  int64_t sequence_{0};
};
}  // namespace rm_aim_sim_bridge
