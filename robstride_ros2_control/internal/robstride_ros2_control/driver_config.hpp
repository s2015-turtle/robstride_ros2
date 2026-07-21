#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/logger.hpp>

#include "robstride_driver/config.hpp"

namespace robstride_ros2_control
{

robstride_driver::DriverConfiguration parse_driver_configuration(
  const hardware_interface::HardwareInfo & info, const rclcpp::Logger & logger);

}  // namespace robstride_ros2_control
