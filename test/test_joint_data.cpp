#include <chrono>

#include <gtest/gtest.h>

#include "detail/joint_data.hpp"
#include "detail/protocol.hpp"

namespace rs = robstride_ros2;
namespace detail = robstride_ros2::detail;
using namespace std::chrono_literals;

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
