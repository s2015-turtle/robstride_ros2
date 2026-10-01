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

TEST(JointData, PositionTransformAcceptsEndpointsWithoutWideningJointRange)
{
  for (const double direction : {1.0, -1.0}) {
    for (const double gear : {1.0, 3.0, 10.0, 100.0}) {
      for (const double offset : {0.0, 1.0, -5.0}) {
        rs::JointData joint;
        joint.limits.position_min = -12.566370614;
        joint.limits.position_max = 12.566370614;
        joint.direction = direction;
        joint.gear_ratio = gear;
        joint.position_offset = offset;
        const auto range = joint.joint_position_range();
        ASSERT_TRUE(range);
        const auto low = joint.checked_joint_to_motor_position(range->first);
        const auto high = joint.checked_joint_to_motor_position(range->second);
        ASSERT_TRUE(low);
        ASSERT_TRUE(high);
        EXPECT_GE(*low, joint.limits.position_min);
        EXPECT_LE(*low, joint.limits.position_max);
        EXPECT_GE(*high, joint.limits.position_min);
        EXPECT_LE(*high, joint.limits.position_max);
        EXPECT_FALSE(joint.checked_joint_to_motor_position(std::nextafter(
          range->first, -std::numeric_limits<double>::infinity())));
        EXPECT_FALSE(joint.checked_joint_to_motor_position(std::nextafter(
          range->second, std::numeric_limits<double>::infinity())));
        EXPECT_FALSE(joint.checked_joint_to_motor_position(
          std::numeric_limits<double>::quiet_NaN()));
        EXPECT_FALSE(joint.checked_joint_to_motor_position(
          std::numeric_limits<double>::infinity()));
      }
    }
  }
}

TEST(JointData, PositionTransformSnapsKnownUpperAndLowerRoundoff)
{
  rs::JointData joint;
  joint.limits.position_min = -12.566370614;
  joint.limits.position_max = 12.566370614;
  joint.gear_ratio = 10.0;
  for (const double direction : {1.0, -1.0}) {
    joint.direction = direction;
    for (const double offset : {1.0, -1.0}) {
      joint.position_offset = offset;
      const double endpoint = offset > 0.0 ? joint.limits.position_max : joint.limits.position_min;
      const double q = direction * endpoint / joint.gear_ratio + offset;
      const auto motor = joint.checked_joint_to_motor_position(q);
      ASSERT_TRUE(motor);
      EXPECT_GE(*motor, joint.limits.position_min);
      EXPECT_LE(*motor, joint.limits.position_max);
    }
  }
  joint.direction = 1.0;
  joint.position_offset = 1.0;
  const auto range = joint.joint_position_range();
  ASSERT_TRUE(range);
  ASSERT_GT((range->second - joint.position_offset) * joint.gear_ratio,
    joint.limits.position_max);
  EXPECT_DOUBLE_EQ(*joint.checked_joint_to_motor_position(range->second),
    joint.limits.position_max);
}

TEST(JointData, PositionTransformRejectsInvalidOrUnrepresentableGeometry)
{
  rs::JointData joint;
  joint.limits.position_min = -12.566370614;
  joint.limits.position_max = 12.566370614;
  joint.position_offset = 1e100;  // Both endpoints collapse to the same joint value.
  EXPECT_FALSE(joint.joint_position_range());
  EXPECT_FALSE(joint.checked_joint_to_motor_position(joint.position_offset));
  joint.position_offset = 0.0;
  joint.gear_ratio = 0.0;
  EXPECT_FALSE(joint.joint_position_range());
  joint.gear_ratio = std::numeric_limits<double>::denorm_min();
  EXPECT_FALSE(joint.joint_position_range());
  joint.gear_ratio = 1.0;
  joint.direction = 0.0;
  EXPECT_FALSE(joint.joint_position_range());
}

TEST(JointData, PositionRoundoffCorrectionHasEightRepresentableStepCap)
{
  rs::JointData joint;
  joint.limits.position_min = -12.566370614;
  joint.limits.position_max = 12.566370614;
  double low = joint.limits.position_min;
  double high = joint.limits.position_max;
  for (int step = 0; step < 8; ++step) {
    low = std::nextafter(low, -std::numeric_limits<double>::infinity());
    high = std::nextafter(high, std::numeric_limits<double>::infinity());
    EXPECT_TRUE(joint.motor_position_within_roundoff(low));
    EXPECT_TRUE(joint.motor_position_within_roundoff(high));
  }
  EXPECT_FALSE(joint.motor_position_within_roundoff(std::nextafter(
    low, -std::numeric_limits<double>::infinity())));
  EXPECT_FALSE(joint.motor_position_within_roundoff(std::nextafter(
    high, std::numeric_limits<double>::infinity())));
  EXPECT_FALSE(joint.motor_position_within_roundoff(
    std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(joint.motor_position_within_roundoff(
    std::numeric_limits<double>::infinity()));
}

TEST(JointData, RejectsAmplifiedCancellationEvenWhenJointEndpointsRemainDistinct)
{
  rs::JointData joint;
  joint.limits.position_min = -12.566370614;
  joint.limits.position_max = 12.566370614;
  joint.gear_ratio = 10.0;
  joint.position_offset = 1e16;
  const double low = joint.limits.position_min / joint.gear_ratio + joint.position_offset;
  const double high = joint.limits.position_max / joint.gear_ratio + joint.position_offset;
  ASSERT_LT(low, high);
  ASSERT_EQ((high - joint.position_offset) * joint.gear_ratio, 20.0);
  EXPECT_FALSE(joint.joint_position_range());
  EXPECT_FALSE(joint.checked_joint_to_motor_position(low));
  EXPECT_FALSE(joint.checked_joint_to_motor_position(high));
}
