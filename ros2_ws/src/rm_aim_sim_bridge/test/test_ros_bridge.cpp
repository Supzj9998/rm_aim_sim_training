#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace fs = std::filesystem;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

struct Process
{
  pid_t pid{-1};
  Process(const std::vector<std::string> & args, const fs::path & log)
  {
    std::vector<char *> argv;
    for (const auto & arg : args) {argv.push_back(const_cast<char *>(arg.c_str()));}
    argv.push_back(nullptr);
    pid = ::fork();
    if (pid == 0) {
      const int output = ::open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
      ::dup2(output, STDOUT_FILENO);
      ::dup2(output, STDERR_FILENO);
      ::close(output);
      ::execv(argv[0], argv.data());
      ::_exit(127);
    }
    if (pid < 0) {throw std::runtime_error("fork failed");}
  }
  void stop()
  {
    if (pid < 0) {return;}
    ::kill(pid, SIGTERM);
    for (int i = 0; i < 100; ++i) {
      if (::waitpid(pid, nullptr, WNOHANG) == pid) {pid = -1; return;}
      std::this_thread::sleep_for(10ms);
    }
    ::kill(pid, SIGKILL);
    ::waitpid(pid, nullptr, 0);
    pid = -1;
  }
  ~Process() {stop();}
};

static Json read_state(const fs::path & directory)
{
  std::ifstream input(directory / "state.json");
  if (!input) {return Json();}
  return Json::parse(input, nullptr, false);
}

TEST(RosBridge, CoordinatesControlGravityAndWatchdogs)
{
  const fs::path root = fs::weakly_canonical(SIM_ROOT);
  const char * env_godot = std::getenv("GODOT_BIN");
  const fs::path godot = env_godot ? fs::path(env_godot) : root / ".tools/godot";
  if (::access(godot.c_str(), X_OK) != 0) {GTEST_SKIP() << "No Godot executable";}
  const std::string token = std::to_string(::getpid());
  const std::string channel = "ros_bridge_test_" + token;
  const std::string ns = "/bridge_test_" + token;
  const fs::path temporary = fs::temp_directory_path() / channel;
  const fs::path mailbox = fs::path("/dev/shm") / ("rm_aim_sim_" + channel);
  fs::create_directories(temporary);
  const auto config = temporary / "scenario.cfg";
  std::ofstream(config) << "[ipc]\nchannel=\"" << channel << "\"\n[weapon]\ndamage=0\n";
  // Isolate DDS and /clock from any running user simulator.
  ::setenv("ROS_DOMAIN_ID", std::to_string(100 + ::getpid() % 100).c_str(), 1);
  {
    Process simulator({godot.string(), "--headless", "--path", root.string(), "--",
      "--remote", "--config=" + config.string()}, temporary / "godot.log");
    Process bridge({BRIDGE_EXECUTABLE, "--ros-args", "-r", "__ns:=" + ns,
      "-r", "/clock:=" + ns + "/clock", "-p", "channel:=" + channel}, temporary / "bridge.log");
    rclcpp::init(0, nullptr);
    struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
    auto node = std::make_shared<rclcpp::Node>("bridge_test_driver");
    const auto qos = rclcpp::QoS(1).reliable();
    std::optional<sensor_msgs::msg::JointState> actual;
    std::optional<geometry_msgs::msg::PoseArray> armors;
    bool clock_received = false;
    auto joints_sub = node->create_subscription<sensor_msgs::msg::JointState>(ns + "/gimbal", qos,
      [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {actual = *msg;});
    auto armors_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(ns + "/armors", qos,
      [&](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {armors = *msg;});
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(ns + "/clock", qos,
      [&](rosgraph_msgs::msg::Clock::ConstSharedPtr) {clock_received = true;});
    auto commands = node->create_publisher<sensor_msgs::msg::JointState>(ns + "/command", qos);
    auto enable = node->create_publisher<std_msgs::msg::Bool>(ns + "/enable", qos);
    auto fire = node->create_publisher<std_msgs::msg::Bool>(ns + "/fire", qos);
    auto spin_for = [&](double seconds, const std::function<void()> & action) {
        const auto end = Clock::now() + std::chrono::duration<double>(seconds);
        while (Clock::now() < end) {
          rclcpp::spin_some(node);
          action();
          std::this_thread::sleep_for(20ms);
        }
      };
    auto wait_for = [&](const std::function<bool()> & condition, double seconds = 8) {
        const auto end = Clock::now() + std::chrono::duration<double>(seconds);
        while (Clock::now() < end) {
          rclcpp::spin_some(node);
          if (condition()) {return true;}
          std::this_thread::sleep_for(20ms);
        }
        return false;
      };
    ASSERT_TRUE(wait_for([&] {return actual && armors && !armors->poses.empty() && clock_received;}))
      << "Logs: " << temporary;
    EXPECT_EQ(armors->header.frame_id, "odom");
    auto state = read_state(mailbox);
    ASSERT_TRUE(state.is_object());
    const auto & position = armors->poses.front().position;
    bool matched = false;
    for (const auto & armor : state["truth"]["armors"]) {
      const auto & center = armor["center"];
      const auto & origin = state["gimbal"]["origin"];
      if (std::abs(position.x - (center[0].get<double>() - origin[0].get<double>())) < 1e-6 &&
        std::abs(position.y - (center[1].get<double>() - origin[1].get<double>())) < 1e-6 &&
        std::abs(position.z - (center[2].get<double>() - origin[2].get<double>())) < 1e-6)
      {matched = true;}
    }
    EXPECT_TRUE(matched);

    auto send = [&](double yaw, double pitch, bool shoot, bool send_enable = true) {
        if (!actual) {return;}
        sensor_msgs::msg::JointState command;
        command.header = actual->header;
        command.name = {"yaw", "pitch"};
        command.position = {yaw, pitch};
        commands->publish(command);
        std_msgs::msg::Bool message;
        if (send_enable) {message.data = true; enable->publish(message);}
        message.data = shoot;
        fire->publish(message);
      };
    spin_for(1.2, [&] {send(.1, .04, false);});
    ASSERT_TRUE(actual);
    EXPECT_NEAR(actual->position[0], .1, .005);
    EXPECT_NEAR(actual->position[1], .04, .005);
    state = read_state(mailbox);
    EXPECT_NEAR(state["gimbal"]["pitch"].get<double>(), -.04, .005);
    EXPECT_EQ(state["stats"]["fired"].get<int>(), 0);

    ASSERT_TRUE(wait_for([&] {return armors && !armors->poses.empty();}));
    const auto target = armors->poses.front().position;
    const double horizontal = std::hypot(target.x, target.y);
    const auto weapon = state["weapon"];
    const double speed = weapon["speed"], gravity = weapon["gravity"], length = weapon["barrel_length"];
    double elevation = std::atan2(target.z, horizontal);
    for (int i = 0; i < 12; ++i) {
      const double d = horizontal - length * std::cos(elevation);
      const double h = target.z - length * std::sin(elevation);
      elevation = std::atan((speed * speed -
        std::sqrt(std::pow(speed, 4) - gravity * (gravity * d * d + 2 * h * speed * speed))) /
        (gravity * d));
    }
    const double yaw = std::atan2(target.y, target.x);
    spin_for(.8, [&] {send(yaw, elevation, false);});
    spin_for(.8, [&] {send(yaw, elevation, true);});
    state = read_state(mailbox);
    EXPECT_GE(state["stats"]["hit"].get<int>(), 3);
    // The bridge remains alive and must stop shooting when the student's streams stop.
    spin_for(.5, [] {});
    state = read_state(mailbox);
    const int fired = state["stats"]["fired"];
    EXPECT_EQ(state["stats"]["pending"].get<int>(), 0);
    spin_for(.3, [] {});
    EXPECT_EQ(read_state(mailbox)["stats"]["fired"].get<int>(), fired);

    // No rearming from angle/fire streams alone after enable has expired.
    spin_for(.4, [&] {send(yaw, elevation, true, false);});
    EXPECT_EQ(read_state(mailbox)["stats"]["fired"].get<int>(), fired);
    simulator.stop();
    ASSERT_TRUE(wait_for([&] {return armors && armors->poses.empty();}, 2));
  }
  fs::remove_all(mailbox);
  fs::remove_all(temporary);
}
