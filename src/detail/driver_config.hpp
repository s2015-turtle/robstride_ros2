#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/logger.hpp>

#include "detail/joint_data.hpp"
#include "detail/can_transport.hpp"

namespace robstride_ros2::detail
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

DriverConfiguration parse_driver_configuration(
  const hardware_interface::HardwareInfo & info, const rclcpp::Logger & logger);

}  // namespace robstride_ros2::detail
