#include <chrono>

#include <gtest/gtest.h>

#include "robstride_driver/joint_data.hpp"
#include "robstride_driver/protocol.hpp"

namespace rs = robstride_driver;
namespace detail = robstride_driver;
using namespace std::chrono_literals;

TEST(CommandLimits, RejectsOutOfRangeAndNonFiniteValues)
{
  const rs::CommandLimits limits{-1.0, 2.0, -3.0, 4.0, -5.0, 6.0};
  EXPECT_FALSE(limits.contains_position(-2.0));
  EXPECT_TRUE(limits.contains_position(-1.0));
  EXPECT_TRUE(limits.contains_position(2.0));
  EXPECT_FALSE(limits.contains_velocity(5.0));
  EXPECT_TRUE(limits.contains_velocity(4.0));
  EXPECT_FALSE(limits.contains_effort(-6.0));
  EXPECT_TRUE(limits.contains_effort(-5.0));
  EXPECT_FALSE(limits.contains_position(std::numeric_limits<double>::quiet_NaN()));
}

TEST(JointData, ConvertsEffortBetweenJointAndMotorCoordinates)
{
  rs::JointData joint;
  joint.gear_ratio = 2.0;

  joint.direction = 1.0;
  EXPECT_DOUBLE_EQ(joint.joint_to_motor_effort(6.0), 3.0);
  EXPECT_DOUBLE_EQ(joint.motor_to_joint_effort(3.0), 6.0);

  joint.direction = -1.0;
  EXPECT_DOUBLE_EQ(joint.joint_to_motor_effort(6.0), -3.0);
  EXPECT_DOUBLE_EQ(joint.motor_to_joint_effort(-3.0), 6.0);
}

TEST(JointData, UnityGearRatioPreservesExistingEffortMapping)
{
  rs::JointData joint;
  joint.gear_ratio = 1.0;

  joint.direction = 1.0;
  EXPECT_DOUBLE_EQ(joint.joint_to_motor_effort(4.0), 4.0);
  EXPECT_DOUBLE_EQ(joint.motor_to_joint_effort(4.0), 4.0);

  joint.direction = -1.0;
  EXPECT_DOUBLE_EQ(joint.joint_to_motor_effort(4.0), -4.0);
  EXPECT_DOUBLE_EQ(joint.motor_to_joint_effort(-4.0), 4.0);
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
