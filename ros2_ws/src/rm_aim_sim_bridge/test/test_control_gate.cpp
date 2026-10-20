#include <gtest/gtest.h>
#include "rm_aim_sim_bridge/control_gate.hpp"
using rm_aim_sim_bridge::ControlGate;
using Json = nlohmann::json;

static Json state()
{
  return {{"publisher_id", "server"}, {"session_id", "one"}, {"sim_time", 2.0},
    {"paused", false}, {"ipc", {{"enabled", true}}}, {"gimbal", {{"origin", {0, 0, .65}}}},
    {"appearance", {{"robot", "infantry_3"}}},
    {"camera", {{"K", {1, 0, 0, 0, 1, 0, 0, 0, 1}}, {"t_camera2gimbal", {.08, 0, .04}}}}};
}

TEST(ControlGate, ConvertsPitchAndRequiresEnable)
{
  ControlGate gate(.25);
  gate.update(state(), 10);
  ASSERT_TRUE(gate.angles({"pitch", "yaw"}, {.1, .2}, 2, "odom", 10));
  EXPECT_FALSE(gate.command(10).control);
  gate.enable(true, 10);
  gate.fire(true, 10);
  EXPECT_TRUE(gate.command(10).fire);
  EXPECT_DOUBLE_EQ(gate.command(10).yaw, .2);
  EXPECT_DOUBLE_EQ(gate.command(10).pitch, -.1);
}

TEST(ControlGate, EnableAndAnglesExpireIndependently)
{
  ControlGate gate(.25);
  auto s = state();
  gate.update(s, 10);
  gate.enable(true, 10);
  gate.angles({"yaw", "pitch"}, {0, 0}, 2, "odom", 10);
  gate.fire(true, 10);
  gate.update(s, 10.3);
  gate.enable(true, 10.3);
  EXPECT_FALSE(gate.command(10.3).control);  // angle stream stopped
  gate.update(s, 11);
  gate.angles({"yaw", "pitch"}, {0, 0}, 2, "odom", 11);
  gate.enable(true, 11);
  gate.update(s, 11.3);
  gate.angles({"yaw", "pitch"}, {0, 0}, 2, "odom", 11.3);
  EXPECT_FALSE(gate.command(11.3).control);  // enable stream stopped
}

TEST(ControlGate, FireExpiresWhileTrackingContinues)
{
  ControlGate gate(.25);
  auto s = state();
  gate.update(s, 10);
  gate.enable(true, 10);
  gate.angles({"yaw", "pitch"}, {0, 0}, 2, "odom", 10);
  gate.fire(true, 10);
  gate.update(s, 10.3);
  gate.enable(true, 10.3);
  gate.angles({"yaw", "pitch"}, {0, 0}, 2, "odom", 10.3);
  EXPECT_TRUE(gate.command(10.3).control);
  EXPECT_FALSE(gate.command(10.3).fire);
}

TEST(ControlGate, PauseSessionChangeAndStaleStateDisarm)
{
  for (int mode = 0; mode < 4; ++mode) {
    ControlGate gate(.25);
    auto s = state();
    gate.update(s, 10);
    gate.enable(true, 10);
    gate.angles({"yaw", "pitch"}, {0, 0}, 2, "odom", 10);
    gate.fire(true, 10);
    if (mode == 0) {s["paused"] = true;}
    if (mode == 1) {s["session_id"] = "two";}
    if (mode == 2) {s["ipc"]["enabled"] = false;}
    if (mode != 3) {gate.update(s, 10.1);}
    EXPECT_FALSE(gate.command(mode == 3 ? 10.3 : 10.1).control);
  }
}

TEST(ControlGate, RejectsOldFutureMalformedAndNanCommands)
{
  ControlGate gate(.25);
  auto s = state();
  gate.update(s, 10);
  EXPECT_FALSE(gate.angles({"yaw", "pitch"}, {0, 0}, 1, "odom", 10));
  EXPECT_FALSE(gate.angles({"yaw", "pitch"}, {0, 0}, 3, "odom", 10));
  EXPECT_FALSE(gate.angles({"yaw", "yaw"}, {0, 0}, 2, "odom", 10));
  EXPECT_FALSE(gate.angles({"yaw", "pitch"}, {0, 0}, 2, "camera", 10));
  EXPECT_FALSE(gate.angles({"yaw", "pitch"},
    {0, std::numeric_limits<double>::quiet_NaN()}, 2, "odom", 10));
  gate.enable(true, 10);
  EXPECT_FALSE(gate.command(10).control);
}
