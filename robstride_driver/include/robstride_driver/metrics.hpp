#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace robstride_driver
{

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
};

struct DriverMetrics
{
  CanTransportMetrics transport;
  uint64_t feedback_frames_received{0};
  uint64_t parameter_frames_received{0};
  std::chrono::nanoseconds observation_period{0};
  std::vector<MotorFeedbackMetrics> motors;

  uint64_t received_frames() const noexcept
  {
    return feedback_frames_received + parameter_frames_received;
  }

  double receive_rate_hz() const noexcept;
};

}  // namespace robstride_driver
