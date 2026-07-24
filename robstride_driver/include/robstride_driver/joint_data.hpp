#pragma once

#include <algorithm>
#include <chrono>
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

  double clamp_position(double value) const noexcept
  {
    return std::clamp(value, position_min, position_max);
  }

  double clamp_velocity(double value) const noexcept
  {
    return std::clamp(value, velocity_min, velocity_max);
  }

  double clamp_effort(double value) const noexcept
  {
    return std::clamp(value, effort_min, effort_max);
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
};

}  // namespace robstride_driver
