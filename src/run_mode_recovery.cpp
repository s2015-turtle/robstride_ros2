#include "robstride_ros2/run_mode_recovery.hpp"

#include "robstride_ros2/protocol.hpp"

namespace robstride_ros2
{

RunModeRecoveryAction update_run_mode_recovery(
  RunModeRecoveryState & state, uint8_t reported_mode,
  std::chrono::steady_clock::time_point now, std::chrono::milliseconds timeout,
  std::chrono::milliseconds retry_interval)
{
  if (reported_mode == kMotorModeRun) {
    if (!state.active) {return RunModeRecoveryAction::none;}
    state = RunModeRecoveryState{};
    return RunModeRecoveryAction::recovered;
  }

  if (!state.active) {
    state.active = true;
    state.detected_mode = reported_mode;
    state.started_at = now;
  }

  if (now - state.started_at >= timeout) {return RunModeRecoveryAction::failed;}

  if (state.attempts == 0 || now - state.last_attempt_at >= retry_interval) {
    state.last_attempt_at = now;
    ++state.attempts;
    return RunModeRecoveryAction::send_enable;
  }
  return RunModeRecoveryAction::none;
}

}  // namespace robstride_ros2
