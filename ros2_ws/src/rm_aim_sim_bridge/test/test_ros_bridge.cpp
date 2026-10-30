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
#include "rm_aim_sim_bridge/msg/aim_command.hpp"
#include "rm_aim_sim_bridge/msg/armor_corners_array.hpp"
#include "rm_aim_sim_bridge/msg/gimbal_state.hpp"
#include "rm_aim_sim_bridge/msg/telemetry.hpp"
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
  std::ofstream(config) << "[ipc]\nchannel=\"" << channel <<
    "\"\n[weapon]\nspeed=25.5\ngravity=9.6\nrate=8.0\ndelay=0.02\ndamage=7\n"
    "[gimbal]\nbarrel_length=0.25\nideal=false\n[target]\nphase=2.356194490192345\n";
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
    using GimbalState = rm_aim_sim_bridge::msg::GimbalState;
    std::optional<GimbalState> actual;
    bool yaw_motion_received = false;
    bool pitch_motion_received = false;
    using ArmorCornersArray = rm_aim_sim_bridge::msg::ArmorCornersArray;
    std::optional<ArmorCornersArray> armors;
    using Telemetry = rm_aim_sim_bridge::msg::Telemetry;
    std::optional<Telemetry> telemetry;
    bool clock_received = false;
    auto gimbal_sub = node->create_subscription<GimbalState>(ns + "/gimbal", qos,
      [&](GimbalState::ConstSharedPtr msg) {
        actual = *msg;
        yaw_motion_received = yaw_motion_received || msg->yaw_velocity > .001;
        pitch_motion_received = pitch_motion_received || msg->pitch_velocity > .001;
      });
    auto armors_sub = node->create_subscription<ArmorCornersArray>(ns + "/armors", qos,
      [&](ArmorCornersArray::ConstSharedPtr msg) {armors = *msg;});
    auto telemetry_sub = node->create_subscription<Telemetry>(
      ns + "/telemetry", qos, [&](Telemetry::ConstSharedPtr msg) {telemetry = *msg;});
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(ns + "/clock", qos,
      [&](rosgraph_msgs::msg::Clock::ConstSharedPtr) {clock_received = true;});
    using AimCommand = rm_aim_sim_bridge::msg::AimCommand;
    auto commands = node->create_publisher<AimCommand>(ns + "/command", qos);
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
    ASSERT_TRUE(wait_for([&] {
      return actual && armors && !armors->armors.empty() && telemetry && clock_received && !armors->session_id.empty();
    }))
      << "Logs: " << temporary;
    EXPECT_EQ(armors->header.frame_id, "odom");
    EXPECT_EQ(actual->header.frame_id, "odom");
    EXPECT_EQ(telemetry->header.frame_id, "odom");
    EXPECT_DOUBLE_EQ(telemetry->speed, 25.5);
    EXPECT_DOUBLE_EQ(telemetry->gravity, 9.6);
    EXPECT_DOUBLE_EQ(telemetry->barrel_length, .25);
    EXPECT_DOUBLE_EQ(telemetry->rate, 8.0);
    EXPECT_DOUBLE_EQ(telemetry->delay, .02);
    auto state = read_state(mailbox);
    ASSERT_TRUE(state.is_object());
    EXPECT_TRUE(state.contains("health") && state.contains("stats"));
    const auto session = armors->session_id;
    EXPECT_EQ(session, state["publisher_id"].get<std::string>() + "/" + state["session_id"].get<std::string>());
    EXPECT_EQ(node->count_publishers(ns + "/session"), 0U);
    ASSERT_EQ(armors->armors.size(), 2U);  // Both facing plates are exposed for student selection.
    EXPECT_EQ(armors->armors.size(), state["camera"]["armors"].size());
    for (const auto & observed : armors->armors) {
      bool matched = false;
      for (const auto & armor : state["truth"]["armors"]) {
        const auto & corners = armor["corners"];
        const auto & origin = state["gimbal"]["origin"];
        bool same = true;
        for (size_t i = 0; i < observed.corners.size(); ++i) {
          same = same &&
            std::abs(observed.corners[i].x - (corners[i][0].get<double>() - origin[0].get<double>())) < 1e-6 &&
            std::abs(observed.corners[i].y - (corners[i][1].get<double>() - origin[1].get<double>())) < 1e-6 &&
            std::abs(observed.corners[i].z - (corners[i][2].get<double>() - origin[2].get<double>())) < 1e-6;
        }
        if (same) {
          bool visible = false;
          for (const auto & observation : state["camera"]["armors"]) {
            visible = visible || observation["id"] == armor["id"];
          }
          EXPECT_TRUE(visible);
          matched = true;
        }
      }
      EXPECT_TRUE(matched);  // Every corner is translated, never rotated into the camera frame.
    }

    auto send = [&](double yaw, double pitch, bool shoot, bool enabled = true) {
        if (!armors) {return;}
        AimCommand command;
        command.header = armors->header;
        command.session_id = armors->session_id;
        command.yaw = yaw;
        command.pitch = pitch;
        command.enable = enabled;
        command.fire = shoot;
        commands->publish(command);
      };
    spin_for(1.2, [&] {send(.1, .04, false);});
    ASSERT_TRUE(actual);
    EXPECT_NEAR(actual->yaw, .1, .005);
    EXPECT_NEAR(actual->pitch, .04, .005);
    // Servo mode exercises nonzero velocity fields and their ROS signs.
    EXPECT_TRUE(yaw_motion_received);
    EXPECT_TRUE(pitch_motion_received);
    EXPECT_NEAR(actual->yaw_velocity, 0.0, .005);
    EXPECT_NEAR(actual->pitch_velocity, 0.0, .005);
    state = read_state(mailbox);
    EXPECT_NEAR(state["gimbal"]["pitch"].get<double>(), -.04, .005);
    EXPECT_EQ(state["stats"]["fired"].get<int>(), 0);

    // An older computation must not replace the current angles or fire decision.
    AimCommand old;
    old.header = actual->header;
    old.header.stamp = rclcpp::Time(old.header.stamp) - rclcpp::Duration::from_seconds(.1);
    old.session_id = session;
    old.yaw = -.6;
    old.pitch = -.2;
    old.enable = old.fire = true;
    spin_for(.12, [&] {commands->publish(old);});
    EXPECT_NEAR(actual->yaw, .1, .005);
    EXPECT_EQ(read_state(mailbox)["stats"]["fired"].get<int>(), 0);

    // A packet with a current timestamp but an old session cannot rearm control.
    spin_for(.3, [&] {
      old.header = actual->header;
      old.session_id = "previous-publisher/previous-session";
      commands->publish(old);
    });
    EXPECT_EQ(read_state(mailbox)["stats"]["fired"].get<int>(), 0);

    ASSERT_TRUE(wait_for([&] {return armors && !armors->armors.empty();}));
    geometry_msgs::msg::Point target;
    for (const auto & corner : armors->armors.front().corners) {
      target.x += corner.x * .25;
      target.y += corner.y * .25;
      target.z += corner.z * .25;
    }
    const double horizontal = std::hypot(target.x, target.y);
    // The student can solve using published ballistic parameters alone.
    const double speed = telemetry->speed, gravity = telemetry->gravity, length = telemetry->barrel_length;
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

    // An atomic disabled command cannot rearm shooting even with fire=true.
    spin_for(.4, [&] {send(yaw, elevation, true, false);});
    EXPECT_EQ(read_state(mailbox)["stats"]["fired"].get<int>(), fired);
    simulator.stop();
    ASSERT_TRUE(wait_for([&] {return armors && armors->armors.empty();}, 2));
    EXPECT_EQ(armors->session_id, session);
  }
  fs::remove_all(mailbox);
  fs::remove_all(temporary);
}
