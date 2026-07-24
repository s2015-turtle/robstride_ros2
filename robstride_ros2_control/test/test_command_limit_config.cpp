#include <gtest/gtest.h>

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

hardware_interface::HardwareInfo hardware_info()
{
  hardware_interface::HardwareInfo hardware;
  hardware.name = "limit_test";

  hardware_interface::ComponentInfo joint;
  joint.name = "joint_1";
  joint.parameters = {
    {"can_id", "1"},
    {"can_timeout_ticks", "4000"},
    {"position_min", "-10.0"},
    {"position_max", "10.0"},
    {"velocity_min", "-20.0"},
    {"velocity_max", "20.0"},
    {"effort_min", "-6.0"},
    {"effort_max", "6.0"},
    {"kp_max", "500.0"},
    {"kd_max", "5.0"},
    {"kp", "30.0"},
    {"kd", "1.0"},
    {"direction", "-1"},
    {"gear_ratio", "2.0"},
    {"position_offset", "0.5"},
  };
  joint.command_interfaces = {
    interface("position"), interface("velocity"), interface("effort")};
  joint.state_interfaces = {
    interface("position"), interface("velocity"), interface("effort")};
  hardware.joints.push_back(joint);
  return hardware;
}
}  // namespace

TEST(CommandLimitConfig, DerivesBackwardCompatibleJointLimits)
{
  const auto configuration = rs::parse_driver_configuration(hardware_info());
  const auto & limits = configuration.joints[0].command_limits;
  EXPECT_DOUBLE_EQ(limits.position_min, -4.5);
  EXPECT_DOUBLE_EQ(limits.position_max, 5.5);
  EXPECT_DOUBLE_EQ(limits.velocity_min, -10.0);
  EXPECT_DOUBLE_EQ(limits.velocity_max, 10.0);
  EXPECT_DOUBLE_EQ(limits.effort_min, -6.0);
  EXPECT_DOUBLE_EQ(limits.effort_max, 6.0);
}

TEST(CommandLimitConfig, AcceptsTighterOperationalLimits)
{
  auto hardware = hardware_info();
  auto & parameters = hardware.joints[0].parameters;
  parameters["command_position_min"] = "-1.0";
  parameters["command_position_max"] = "1.0";
  parameters["command_velocity_min"] = "-2.0";
  parameters["command_velocity_max"] = "2.0";
  parameters["command_effort_min"] = "-3.0";
  parameters["command_effort_max"] = "3.0";

  const auto configuration = rs::parse_driver_configuration(hardware);
  const auto & limits = configuration.joints[0].command_limits;
  EXPECT_DOUBLE_EQ(limits.position_min, -1.0);
  EXPECT_DOUBLE_EQ(limits.velocity_max, 2.0);
  EXPECT_DOUBLE_EQ(limits.effort_min, -3.0);
}

TEST(CommandLimitConfig, RejectsInvalidOrOutOfWireLimits)
{
  auto hardware = hardware_info();
  hardware.joints[0].parameters["command_position_min"] = "-6.0";
  hardware.joints[0].parameters["command_position_max"] = "1.0";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);

  hardware = hardware_info();
  hardware.joints[0].parameters["command_velocity_min"] = "nan";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);

  hardware = hardware_info();
  hardware.joints[0].parameters["command_effort_min"] = "2.0";
  hardware.joints[0].parameters["command_effort_max"] = "-2.0";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
}
