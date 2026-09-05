#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace robstride_driver
{

enum class CanTransportHealthState
{
  healthy,
  bridge_unavailable,
  transmit_stalled,
  worker_stopped,
};

struct CanTransportHealth
{
  CanTransportHealthState state{CanTransportHealthState::healthy};
  std::chrono::nanoseconds duration{0};
  bool persistent{false};
};

struct MotorFeedbackSample
{
  uint64_t count{0};
  int64_t last_received_at_ns{0};
  int64_t maximum_gap_ns{0};
  uint8_t mode{0};
  uint8_t fault_flags{0};
  double temperature{std::numeric_limits<double>::quiet_NaN()};
};

class AtomicMotorFeedback
{
public:
  // One receive thread stores samples; any number of diagnostic readers may load them.
  AtomicMotorFeedback() noexcept;
  void store(const MotorFeedbackSample & sample) noexcept;
  MotorFeedbackSample load() const noexcept;

private:
  std::atomic<uint64_t> sequence_{0};
  std::atomic<uint64_t> count_{0};
  std::atomic<int64_t> last_received_at_ns_{0};
  std::atomic<int64_t> maximum_gap_ns_{0};
  std::atomic<uint8_t> mode_{0};
  std::atomic<uint8_t> fault_flags_{0};
  std::atomic<uint64_t> temperature_bits_{0};
};

const char * motor_mode_name(uint8_t mode) noexcept;
const char * transport_health_name(CanTransportHealthState state) noexcept;
std::vector<std::string> motor_fault_names(uint8_t fault_flags);
std::string motor_fault_summary(uint8_t fault_flags);

struct CanTransportMetrics
{
  uint64_t motion_frames_transmitted{0};
  uint64_t recovery_frames_transmitted{0};
  uint64_t transaction_frames_transmitted{0};
  uint64_t motion_frames_coalesced{0};
  std::chrono::nanoseconds observation_period{0};

  uint64_t transmitted_frames() const noexcept
  {
    return motion_frames_transmitted + recovery_frames_transmitted +
           transaction_frames_transmitted;
  }

  double transmit_rate_hz() const noexcept;
  double motion_rate_hz() const noexcept;
};

struct MotorFeedbackMetrics
{
  std::string joint_name;
  uint8_t can_id{0};
  uint64_t feedback_frames_received{0};
  bool feedback_received{false};
  std::chrono::nanoseconds current_feedback_age{0};
  std::chrono::nanoseconds maximum_feedback_age{0};
  double feedback_rate_hz{0.0};
  double temperature{std::numeric_limits<double>::quiet_NaN()};
  uint8_t mode{0};
  uint8_t fault_flags{0};
  bool feedback_stale{false};
  bool recovery_active{false};
  uint64_t recovery_attempts{0};
};

struct DriverMetrics
{
  CanTransportMetrics transport;
  uint64_t feedback_frames_received{0};
  uint64_t parameter_frames_received{0};
  std::chrono::nanoseconds observation_period{0};
  bool hardware_active{false};
  CanTransportHealth transport_health{};
  std::vector<MotorFeedbackMetrics> motors;

  uint64_t received_frames() const noexcept
  {
    return feedback_frames_received + parameter_frames_received;
  }

  double receive_rate_hz() const noexcept;
};

}  // namespace robstride_driver
