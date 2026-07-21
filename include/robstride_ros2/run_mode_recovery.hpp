#pragma once

#include <chrono>
#include <cstdint>

namespace robstride_ros2
{

enum class RunModeRecoveryAction
{
  none,
  send_enable,
  recovered,
  failed,
};

struct RunModeRecoveryState
{
  bool active{false};
  uint8_t detected_mode{0};
  int attempts{0};
  std::chrono::steady_clock::time_point started_at{};
  std::chrono::steady_clock::time_point last_attempt_at{};
};

RunModeRecoveryAction update_run_mode_recovery(
  RunModeRecoveryState & state, uint8_t reported_mode,
  std::chrono::steady_clock::time_point now, std::chrono::milliseconds timeout,
  std::chrono::milliseconds retry_interval);

}  // namespace robstride_ros2
