#include <chrono>

#include <gtest/gtest.h>

#include "robstride_driver/joint_data.hpp"
#include "robstride_driver/protocol.hpp"

namespace rs = robstride_driver;
namespace detail = robstride_driver;
using namespace std::chrono_literals;

TEST(CommandLimits, ClampsPositionVelocityAndEffort)
{
  const rs::CommandLimits limits{-1.0, 2.0, -3.0, 4.0, -5.0, 6.0};
  EXPECT_DOUBLE_EQ(limits.clamp_position(-2.0), -1.0);
  EXPECT_DOUBLE_EQ(limits.clamp_position(1.0), 1.0);
  EXPECT_DOUBLE_EQ(limits.clamp_velocity(5.0), 4.0);
  EXPECT_DOUBLE_EQ(limits.clamp_effort(-6.0), -5.0);
}

TEST(RecoveryState, IgnoresRunModeWhenHealthy)
{
  detail::RecoveryState state;
  const auto now = std::chrono::steady_clock::time_point{};
  EXPECT_EQ(
    state.update(rs::kMotorModeRun, now, 500ms, 100ms), detail::RecoveryAction::none);
  EXPECT_FALSE(state.active);
}

TEST(RecoveryState, RetriesAtConfiguredIntervalAndRecovers)
{
  detail::RecoveryState state;
  const auto start = std::chrono::steady_clock::time_point{};

  EXPECT_EQ(
    state.update(rs::kMotorModeReset, start, 500ms, 100ms),
    detail::RecoveryAction::send_enable);
  EXPECT_EQ(state.attempts, 1);
  EXPECT_EQ(
    state.update(rs::kMotorModeReset, start + 99ms, 500ms, 100ms),
    detail::RecoveryAction::none);
  EXPECT_EQ(
    state.update(rs::kMotorModeReset, start + 100ms, 500ms, 100ms),
    detail::RecoveryAction::send_enable);
  EXPECT_EQ(state.attempts, 2);
  EXPECT_EQ(
    state.update(rs::kMotorModeRun, start + 101ms, 500ms, 100ms),
    detail::RecoveryAction::recovered);
  EXPECT_FALSE(state.active);
}

TEST(RecoveryState, FailsAfterRecoveryTimeout)
{
  detail::RecoveryState state;
  const auto start = std::chrono::steady_clock::time_point{};
  EXPECT_EQ(
    state.update(rs::kMotorModeReset, start, 500ms, 100ms),
    detail::RecoveryAction::send_enable);
  EXPECT_EQ(
    state.update(rs::kMotorModeReset, start + 500ms, 500ms, 100ms),
    detail::RecoveryAction::failed);
}
