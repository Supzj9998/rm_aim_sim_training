#include "rm_aim_sim_bridge/mailbox.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <regex>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace rm_aim_sim_bridge
{
namespace
{
struct File
{
  int fd;
  ~File() {if (fd >= 0) {::close(fd);}}
};
}
Mailbox::Mailbox(const std::string & channel)
{
  if (!std::regex_match(channel, std::regex("[A-Za-z0-9_-]{1,80}"))) {
    throw std::invalid_argument("Invalid IPC channel");
  }
  path_ = "/dev/shm/rm_aim_sim_" + channel;
}
Mailbox::~Mailbox()
{
  try {send();} catch (...) {}
  if (lock_ >= 0) {::close(lock_);}
  if (directory_ >= 0) {::close(directory_);}
}
std::optional<Json> Mailbox::poll()
{
  if (directory_ < 0) {
    File directory{::open(path_.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW)};
    if (directory.fd < 0) {
      if (errno == ENOENT) {return std::nullopt;}
      throw std::runtime_error("Cannot open IPC directory");
    }
    struct stat info{};
    if (::fstat(directory.fd, &info) || info.st_uid != ::getuid() || (info.st_mode & 0777) != 0700) {
      throw std::runtime_error("IPC directory must belong to this UID with mode 0700");
    }
    File lock{::openat(directory.fd, "client.lock", O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600)};
    if (lock.fd < 0 || ::flock(lock.fd, LOCK_EX | LOCK_NB)) {
      throw std::runtime_error("IPC client.lock occupied; stop other clients on this channel");
    }
    directory_ = std::exchange(directory.fd, -1);
    lock_ = std::exchange(lock.fd, -1);
  }
  File input{::openat(directory_, "state.json", O_RDONLY | O_CLOEXEC | O_NOFOLLOW)};
  if (input.fd < 0) {
    if (errno == ENOENT) {return std::nullopt;}
    throw std::runtime_error("Cannot open state.json");
  }
  struct stat info{};
  if (::fstat(input.fd, &info) || !S_ISREG(info.st_mode) || info.st_size <= 0 || info.st_size > 65536) {
    return std::nullopt;
  }
  std::string bytes(static_cast<size_t>(info.st_size), '\0');
  size_t offset = 0;
  while (offset < bytes.size()) {
    const auto count = ::read(input.fd, bytes.data() + offset, bytes.size() - offset);
    if (count < 0 && errno == EINTR) {continue;}
    if (count <= 0) {return std::nullopt;}
    offset += static_cast<size_t>(count);
  }
  auto state = Json::parse(bytes, nullptr, false);
  if (!state.is_object() || state.value("protocol", "") != "rm_aim_sim/2" ||
    state.value("type", "") != "state") {return std::nullopt;}
  const auto wall = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  const double age = wall - state.at("wall_unix_sec").get<double>();
  const Json identity = Json::array({state.at("publisher_id"), state.at("publication")});
  if (!std::isfinite(age) || age < -.1 || age > .25 || identity == identity_) {return std::nullopt;}
  state_ = state;
  identity_ = identity;
  return state;
}
void Mailbox::send(bool control, bool fire, double yaw, double pitch)
{
  if (!connected()) {return;}
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {throw std::invalid_argument("Nonfinite command");}
  const Json context = Json::array({state_.at("publisher_id"), state_.at("session_id")});
  if (context != context_) {sequence_ = 0; context_ = context;}
  sequence_ = std::max(sequence_, state_.at("ipc").at("last_seq").get<int64_t>() + 1);
  if (sequence_ > 9007199254740991LL) {throw std::runtime_error("Command sequence exhausted");}
  const Json packet = {{"protocol", "rm_aim_sim/2"}, {"type", "command"},
    {"publisher_id", context[0]}, {"session_id", context[1]}, {"seq", sequence_++},
    {"control", control}, {"fire", fire}, {"yaw", yaw}, {"pitch", pitch}};
  const auto bytes = packet.dump();
  const std::string temporary = "command.bridge-" + std::to_string(::getpid()) + ".tmp";
  File output{::openat(directory_, temporary.c_str(),
      O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600)};
  if (output.fd < 0) {throw std::runtime_error("Cannot create temporary command");}
  try {
    size_t offset = 0;
    while (offset < bytes.size()) {
      const auto count = ::write(output.fd, bytes.data() + offset, bytes.size() - offset);
      if (count < 0 && errno == EINTR) {continue;}
      if (count <= 0) {throw std::runtime_error("Cannot write command");}
      offset += static_cast<size_t>(count);
    }
    if (::renameat(directory_, temporary.c_str(), directory_, "command.json")) {
      throw std::runtime_error("Cannot publish command");
    }
  } catch (...) {
    ::unlinkat(directory_, temporary.c_str(), 0);
    throw;
  }
}
}  // namespace rm_aim_sim_bridge
