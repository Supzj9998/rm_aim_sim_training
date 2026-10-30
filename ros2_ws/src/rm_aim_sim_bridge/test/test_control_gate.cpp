#include <gtest/gtest.h>
#include "rm_aim_sim_bridge/control_gate.hpp"
using rm_aim_sim_bridge::ControlGate;
using Json = nlohmann::json;

static Json state(double time = 2.0)
{
  return {{"publisher_id", "server"}, {"session_id", "one"}, {"sim_time", time},
    {"paused", false}, {"ipc", {{"enabled", true}}}};
}

TEST(ControlGate, AtomicAnglesEnableFireAndPitchConversion)
{
  ControlGate gate(.25);
  gate.update(state(), 10);
  ASSERT_TRUE(gate.accept("server/one", 2, "odom", .2, .1, true, true, 10));
  EXPECT_TRUE(gate.command(10).control);
  EXPECT_TRUE(gate.command(10).fire);
  EXPECT_DOUBLE_EQ(gate.command(10).yaw, .2);
  EXPECT_DOUBLE_EQ(gate.command(10).pitch, -.1);
  ASSERT_TRUE(gate.accept("server/one", 2, "odom", .2, .1, true, false, 10.01));
  EXPECT_TRUE(gate.command(10.01).control);
  EXPECT_FALSE(gate.command(10.01).fire);
  ASSERT_TRUE(gate.accept("server/one", 2, "odom", .2, .1, false, true, 10.02));
  EXPECT_FALSE(gate.command(10.02).control);
  EXPECT_FALSE(gate.command(10.02).fire);
}

TEST(ControlGate, OutOfOrderPacketsPreserveCurrentDecisionAndDoNotRefreshTimeout)
{
  ControlGate gate(.25);
  gate.update(state(), 10);
  gate.update(state(2.2), 10.2);
  ASSERT_TRUE(gate.accept("server/one", 2.2, "odom", .2, .1, true, true, 10.2));
  EXPECT_FALSE(gate.accept("server/one", 2.05, "odom", 1, 0, false, false, 10.21));
  EXPECT_TRUE(gate.command(10.21).fire);
  EXPECT_DOUBLE_EQ(gate.command(10.21).yaw, .2);
  // An equal observation timestamp is allowed for repeated decisions on one frame.
  EXPECT_TRUE(gate.accept("server/one", 2.2, "odom", .3, 0, true, false, 10.22));
  EXPECT_DOUBLE_EQ(gate.command(10.22).yaw, .3);
  gate.update(state(2.4), 10.48);
  EXPECT_FALSE(gate.accept("server/one", 2.19, "odom", 1, 0, true, true, 10.48));
  EXPECT_FALSE(gate.command(10.48).control);
}

TEST(ControlGate, DisableAndTimeoutDoNotResetObservationWatermark)
{
  for (bool disable : {true, false}) {
    ControlGate gate(.25);
    gate.update(state(), 10);
    gate.update(state(2.2), 10.1);
    ASSERT_TRUE(gate.accept("server/one", 2.2, "odom", .2, 0, true, true, 10.1));
    if (disable) {
      ASSERT_TRUE(gate.accept("server/one", 2.2, "odom", 0, 0, false, false, 10.11));
    } else {
      EXPECT_FALSE(gate.command(10.4).control);
    }
    gate.update(state(2.2), 10.41);
    EXPECT_FALSE(gate.accept("server/one", 2.1, "odom", 1, 0, true, true, 10.41));
    EXPECT_FALSE(gate.command(10.41).control);
  }
}

TEST(ControlGate, WallClockAndObservationAgeExpireIndependently)
{
  ControlGate gate(.25);
  gate.update(state(), 10);
  ASSERT_TRUE(gate.accept("server/one", 2, "odom", 0, 0, true, true, 10));
  gate.update(state(), 10.3);
  EXPECT_FALSE(gate.command(10.3).control);  // paused clock does not stop receipt timeout
  ASSERT_TRUE(gate.accept("server/one", 2, "odom", 0, 0, true, true, 10.3));
  gate.update(state(2.3), 10.31);
  EXPECT_FALSE(gate.command(10.31).control);  // recently received old observation
}

TEST(ControlGate, SessionAndPublisherChangesRejectOldPacketsEvenAtSameStamp)
{
  for (bool publisher : {true, false}) {
    ControlGate gate(.25);
    auto s = state();
    gate.update(s, 10);
    ASSERT_TRUE(gate.accept("server/one", 2, "odom", .1, 0, true, true, 10));
    s[publisher ? "publisher_id" : "session_id"] = "two";
    gate.update(s, 10.1);
    EXPECT_FALSE(gate.command(10.1).control);
    EXPECT_FALSE(gate.accept("server/one", 2, "odom", 1, 0, true, true, 10.1));
    const auto session = publisher ? "two/one" : "server/two";
    ASSERT_TRUE(gate.accept(session, 2, "odom", .2, 0, true, false, 10.11));
    EXPECT_DOUBLE_EQ(gate.command(10.11).yaw, .2);
    EXPECT_FALSE(gate.accept("server/one", 2, "odom", 1, 0, true, true, 10.12));
    EXPECT_DOUBLE_EQ(gate.command(10.12).yaw, .2);
  }
}

TEST(ControlGate, ResetCanAcceptLowerStampOnlyInNewSession)
{
  ControlGate gate(.25);
  gate.update(state(), 10);
  gate.accept("server/one", 2, "odom", 0, 0, true, true, 10);
  auto s = state(0);
  s["session_id"] = "reset";
  gate.update(s, 10.1);
  EXPECT_FALSE(gate.accept("server/one", 0, "odom", 1, 0, true, true, 10.1));
  EXPECT_TRUE(gate.accept("server/reset", 0, "odom", 0, 0, true, false, 10.1));
}

TEST(ControlGate, PauseDisabledModeAndLostStateDisarm)
{
  for (int mode = 0; mode < 3; ++mode) {
    ControlGate gate(.25);
    auto s = state();
    gate.update(s, 10);
    gate.accept("server/one", 2, "odom", 0, 0, true, true, 10);
    if (mode == 0) {s["paused"] = true;}
    if (mode == 1) {s["ipc"]["enabled"] = false;}
    if (mode != 2) {gate.update(s, 10.1);}
    const auto now = mode == 2 ? 10.3 : 10.1;
    EXPECT_FALSE(gate.command(now).control);
    EXPECT_FALSE(gate.accept("server/one", 2, "odom", 1, 0, true, true, now));
  }
}

TEST(ControlGate, RejectsOldFutureWrongFrameAndNonfiniteCommands)
{
  ControlGate gate(.25);
  gate.update(state(), 10);
  EXPECT_FALSE(gate.accept("server/one", 1, "odom", 0, 0, true, true, 10));
  EXPECT_FALSE(gate.accept("server/one", 3, "odom", 0, 0, true, true, 10));
  EXPECT_FALSE(gate.accept("server/one", 2, "camera", 0, 0, true, true, 10));
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gate.accept("server/one", nan, "odom", 0, 0, true, true, 10));
  EXPECT_FALSE(gate.accept("server/one", 2, "odom", nan, 0, true, true, 10));
  EXPECT_FALSE(gate.accept("server/one", 2, "odom", 0, nan, true, true, 10));
  EXPECT_FALSE(gate.accept("server/one", 2, "odom", 1001, 0, true, true, 10));
  EXPECT_FALSE(gate.command(10).control);
}
