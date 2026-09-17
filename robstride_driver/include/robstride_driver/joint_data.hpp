#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "robstride_driver/protocol.hpp"

namespace robstride_driver
{

struct StateValues
{
  double position{std::numeric_limits<double>::quiet_NaN()};
  double velocity{std::numeric_limits<double>::quiet_NaN()};
  double effort{std::numeric_limits<double>::quiet_NaN()};
  double temperature{std::numeric_limits<double>::quiet_NaN()};
  double fault{0.0};
};

struct CommandValues
{
  double position{std::numeric_limits<double>::quiet_NaN()};
  double velocity{0.0};
  double effort{0.0};
};

struct CommandLimits
{
  double position_min{0.0};
  double position_max{0.0};
  double velocity_min{0.0};
  double velocity_max{0.0};
  double effort_min{0.0};
  double effort_max{0.0};

  bool contains_position(double value) const noexcept
  {
    return std::isfinite(value) && position_min <= value && value <= position_max;
  }

  bool contains_velocity(double value) const noexcept
  {
    return std::isfinite(value) && velocity_min <= value && value <= velocity_max;
  }

  bool contains_effort(double value) const noexcept
  {
    return std::isfinite(value) && effort_min <= value && value <= effort_max;
  }
};

struct ClaimedInterfaces
{
  bool position{false};
  bool velocity{false};
  bool effort{false};
};

struct FeedbackStatus
{
  bool received{false};
  uint8_t mode{0};
  std::chrono::steady_clock::time_point timestamp{};
};

struct ParameterStatus
{
  bool received{false};
  uint16_t index{0};
  uint32_t value{0};
  std::chrono::steady_clock::time_point timestamp{};
};

enum class RecoveryAction
{
  none,
  send_enable,
  recovered,
  failed,
};

struct RecoveryState
{
  bool active{false};
  uint8_t detected_mode{0};
  int attempts{0};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point last_attempt_at{};

  RecoveryAction update(
    uint8_t reported_mode, std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds timeout, std::chrono::milliseconds retry_interval)
  {
    if (reported_mode == kMotorModeRun) {
      if (!active) {return RecoveryAction::none;}
      *this = RecoveryState{};
      return RecoveryAction::recovered;
    }
    if (!active) {
      active = true;
      detected_mode = reported_mode;
      started_at = now;
    }
    if (now - started_at >= timeout) {return RecoveryAction::failed;}
    if (attempts == 0 || now - last_attempt_at >= retry_interval) {
      last_attempt_at = now;
      ++attempts;
      return RecoveryAction::send_enable;
    }
    return RecoveryAction::none;
  }
};

struct JointData
{
  std::string name;
  uint8_t can_id{0};
  Limits limits{};
  double direction{1.0};
  double gear_ratio{1.0};
  double position_offset{0.0};
  double kp{0.0};
  double kd{0.0};
  uint32_t can_timeout_ticks{0};
  CommandLimits command_limits{};
  bool exports_temperature{false};
  bool exports_fault{false};
  StateValues state{};
  StateValues feedback{};
  CommandValues command{};
  ClaimedInterfaces claimed{};
  FeedbackStatus feedback_status{};
  ParameterStatus parameter_status{};
  RecoveryState recovery{};

  // Every clamp/encoding range used by a neutral command must preserve zero.
  // Position is intentionally excluded: its origin need not be in the operating range.
  const char * invalid_neutral_range() const noexcept
  {
    const auto valid = [](double minimum, double maximum) {
        return std::isfinite(minimum) && std::isfinite(maximum) &&
               minimum < maximum && minimum <= 0.0 && maximum >= 0.0;
      };
    if (!valid(command_limits.velocity_min, command_limits.velocity_max)) {
      return "command_velocity_min/max";
    }
    if (!valid(command_limits.effort_min, command_limits.effort_max)) {
      return "command_effort_min/max";
    }
    if (!valid(limits.velocity_min, limits.velocity_max)) {return "velocity_min/max";}
    if (!valid(limits.effort_min, limits.effort_max)) {return "effort_min/max";}
    if (!valid(limits.effort_wire_min, limits.effort_wire_max)) {
      return "effort_wire_min/max";
    }
    return nullptr;
  }

  double joint_to_motor_effort(double joint_effort) const noexcept
  {
    return direction * joint_effort / gear_ratio;
  }

  double motor_to_joint_effort(double motor_effort) const noexcept
  {
    return direction * motor_effort * gear_ratio;
  }
};

}  // namespace robstride_driver
