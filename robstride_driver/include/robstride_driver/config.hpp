#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include "robstride_driver/can_transport.hpp"
#include "robstride_driver/joint_data.hpp"

namespace robstride_driver
{

struct DriverSettings
{
  uint8_t host_id{0xfd};
  CanTransportOptions transport{};
  std::chrono::milliseconds feedback_timeout{3000};
  bool fail_on_feedback_timeout{true};
  std::chrono::milliseconds recovery_timeout{500};
  std::chrono::milliseconds recovery_retry_interval{100};
  bool clear_faults_on_start{true};
  bool set_zero_on_start{false};
  int stop_repetitions{3};
  std::chrono::milliseconds stop_interval{20};
  std::chrono::milliseconds stop_confirmation_timeout{300};
  std::chrono::milliseconds connection_timeout{3000};
  std::chrono::milliseconds startup_confirmation_timeout{500};
  int startup_retries{3};
};

struct DriverConfiguration
{
  DriverSettings settings;
  std::vector<JointData> joints;
};

}  // namespace robstride_driver
