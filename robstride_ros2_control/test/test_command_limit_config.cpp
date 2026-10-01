#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
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

TEST(CommandLimitConfig, DerivesJointLimitsFromMotorRanges)
{
  const auto configuration = rs::parse_driver_configuration(hardware_info());
  const auto & limits = configuration.joints[0].command_limits;
  EXPECT_DOUBLE_EQ(limits.position_min, -4.5);
  EXPECT_DOUBLE_EQ(limits.position_max, 5.5);
  EXPECT_DOUBLE_EQ(limits.velocity_min, -10.0);
  EXPECT_DOUBLE_EQ(limits.velocity_max, 10.0);
  EXPECT_DOUBLE_EQ(limits.effort_min, -12.0);
  EXPECT_DOUBLE_EQ(limits.effort_max, 12.0);
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

  hardware = hardware_info();
  hardware.joints[0].parameters["command_effort_min"] = "-12.0";
  hardware.joints[0].parameters["command_effort_max"] = "14.0";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
}

TEST(CommandLimitConfig, DerivesOrderedEffortLimitsForNegativeDirection)
{
  auto hardware = hardware_info();
  auto & parameters = hardware.joints[0].parameters;
  parameters["effort_min"] = "-4.0";
  parameters["effort_max"] = "6.0";
  parameters["effort_wire_min"] = "-5.0";
  parameters["effort_wire_max"] = "7.0";

  const auto configuration = rs::parse_driver_configuration(hardware);
  const auto & limits = configuration.joints[0].command_limits;
  EXPECT_DOUBLE_EQ(limits.effort_min, -12.0);
  EXPECT_DOUBLE_EQ(limits.effort_max, 8.0);
}

TEST(CommandLimitConfig, AcceptsRoundedDefaultsAndRejectsAdjacentOutsidePositions)
{
  const auto number = [](double value) {
      std::ostringstream output;
      output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
      return output.str();
    };
  for (const double direction : {1.0, -1.0}) {
    for (const double offset : {1.0, -1.0}) {
      auto hardware = hardware_info();
      auto & parameters = hardware.joints[0].parameters;
      parameters["position_min"] = "-12.566370614";
      parameters["position_max"] = "12.566370614";
      parameters["gear_ratio"] = "10";
      parameters["direction"] = number(direction);
      parameters["position_offset"] = number(offset);
      const auto configuration = rs::parse_driver_configuration(hardware);
      const auto & limits = configuration.joints[0].command_limits;
      parameters["command_position_min"] = number(limits.position_min);
      parameters["command_position_max"] = number(limits.position_max);
      EXPECT_NO_THROW(rs::parse_driver_configuration(hardware));
      parameters["command_position_min"] = number(std::nextafter(
        limits.position_min, -std::numeric_limits<double>::infinity()));
      EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
      parameters["command_position_min"] = number(limits.position_min);
      parameters["command_position_max"] = number(std::nextafter(
        limits.position_max, std::numeric_limits<double>::infinity()));
      EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
    }
  }
}

TEST(CommandLimitConfig, RejectsCollapsedPositionTransform)
{
  auto hardware = hardware_info();
  hardware.joints[0].parameters["position_offset"] = "1e100";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
}

TEST(CommandLimitConfig, RejectsNoncollapsedTransformWithLargeBoundaryError)
{
  auto hardware = hardware_info();
  hardware.joints[0].parameters["position_offset"] = "1e16";
  hardware.joints[0].parameters["gear_ratio"] = "10";
  hardware.joints[0].parameters["position_min"] = "-12.566370614";
  hardware.joints[0].parameters["position_max"] = "12.566370614";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
}
