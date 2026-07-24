#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <hardware_interface/system_interface.hpp>

#include "robstride_ros2_control/driver_config.hpp"

namespace rs = robstride_ros2_control;

namespace
{
hardware_interface::InterfaceInfo interface(const std::string & name)
{
  hardware_interface::InterfaceInfo result;
  result.name = name;
  return result;
}

hardware_interface::HardwareInfo valid_hardware_info()
{
  hardware_interface::HardwareInfo hardware;
  hardware.name = "test_system";

  hardware_interface::ComponentInfo joint;
  joint.name = "joint_1";
  joint.parameters = {
    {"can_id", "1"},
    {"can_timeout_ticks", "4000"},
    {"position_min", "-12.566370614"},
    {"position_max", "12.566370614"},
    {"velocity_min", "-50.0"},
    {"velocity_max", "50.0"},
    {"effort_min", "-6.0"},
    {"effort_max", "6.0"},
    {"kp_max", "500.0"},
    {"kd_max", "5.0"},
    {"kp", "30.0"},
    {"kd", "1.0"},
  };
  joint.command_interfaces = {
    interface("position"), interface("velocity"), interface("effort")};
  joint.state_interfaces = {
    interface("position"), interface("velocity"), interface("effort")};
  hardware.joints.push_back(joint);
  return hardware;
}
}  // namespace

TEST(DriverConfig, AcceptsValidBoundaryWatchdog)
{
  auto hardware = valid_hardware_info();
  hardware.joints[0].parameters["can_timeout_ticks"] =
    std::to_string(std::numeric_limits<uint32_t>::max());
  EXPECT_NO_THROW(rs::parse_driver_configuration(hardware));
}

TEST(DriverConfig, RejectsNonFiniteJointValues)
{
  for (const auto * key : {"kp", "kd", "position_offset", "position_min", "effort_wire_max"}) {
    auto hardware = valid_hardware_info();
    hardware.joints[0].parameters[key] = "nan";
    EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error) << key;
  }

  auto hardware = valid_hardware_info();
  hardware.joints[0].parameters["kd"] = "inf";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
}

TEST(DriverConfig, RejectsInvalidWatchdogValues)
{
  for (const auto * value : {"0", "4294967296", "4000ticks"}) {
    auto hardware = valid_hardware_info();
    hardware.joints[0].parameters["can_timeout_ticks"] = value;
    EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error) << value;
  }
}

TEST(DriverConfig, RejectsBlankCanTopics)
{
  auto hardware = valid_hardware_info();
  hardware.hardware_parameters["can_tx_topic"] = " ";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);

  hardware = valid_hardware_info();
  hardware.hardware_parameters["can_rx_topic"] = "";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
}
