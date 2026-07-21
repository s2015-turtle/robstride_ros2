#include <chrono>

#include <gtest/gtest.h>

#include "robstride_ros2/protocol.hpp"
#include "robstride_ros2/run_mode_recovery.hpp"

namespace rs = robstride_ros2;
using namespace std::chrono_literals;

TEST(RunModeRecovery, IgnoresRunModeWhenHealthy)
{
  rs::RunModeRecoveryState state;
  const auto now = std::chrono::steady_clock::time_point{};
  EXPECT_EQ(
    rs::update_run_mode_recovery(state, rs::kMotorModeRun, now, 500ms, 100ms),
    rs::RunModeRecoveryAction::none);
  EXPECT_FALSE(state.active);
}

TEST(RunModeRecovery, RetriesAtConfiguredIntervalAndRecovers)
{
  rs::RunModeRecoveryState state;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(
    rs::update_run_mode_recovery(state, rs::kMotorModeReset, start, 500ms, 100ms),
    rs::RunModeRecoveryAction::send_enable);
  EXPECT_EQ(state.attempts, 1);
  EXPECT_EQ(
    rs::update_run_mode_recovery(state, rs::kMotorModeReset, start + 99ms, 500ms, 100ms),
    rs::RunModeRecoveryAction::none);
  EXPECT_EQ(
    rs::update_run_mode_recovery(state, rs::kMotorModeReset, start + 100ms, 500ms, 100ms),
    rs::RunModeRecoveryAction::send_enable);
  EXPECT_EQ(state.attempts, 2);
  EXPECT_EQ(
    rs::update_run_mode_recovery(state, rs::kMotorModeRun, start + 101ms, 500ms, 100ms),
    rs::RunModeRecoveryAction::recovered);
  EXPECT_FALSE(state.active);
}

TEST(RunModeRecovery, FailsAfterRecoveryTimeout)
{
  rs::RunModeRecoveryState state;
  const auto start = std::chrono::steady_clock::time_point{};
  EXPECT_EQ(
    rs::update_run_mode_recovery(state, rs::kMotorModeReset, start, 500ms, 100ms),
    rs::RunModeRecoveryAction::send_enable);
  EXPECT_EQ(
    rs::update_run_mode_recovery(state, rs::kMotorModeReset, start + 500ms, 500ms, 100ms),
    rs::RunModeRecoveryAction::failed);
}
