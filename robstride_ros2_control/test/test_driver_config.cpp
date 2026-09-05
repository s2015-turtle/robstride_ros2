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

TEST(DriverConfig, ResolvesAllKnownModelsWithoutNumericLimits)
{
  struct Expected {const char * name; double velocity; double effort; double kp; double kd;};
  const Expected models[] = {
    {"RS00", 33, 14, 500, 5}, {"RS01", 44, 17, 500, 5},
    {"RS02", 44, 17, 500, 5}, {"RS03", 20, 60, 5000, 100},
    {"RS04", 15, 120, 5000, 100}, {"RS05", 50, 5.5, 500, 5},
    {"RS06", 50, 36, 5000, 100}, {"EL05", 50, 6, 500, 5},
    {"EduLite05", 50, 6, 500, 5}};
  for (const auto & model : models) {
    auto hardware = valid_hardware_info();
    hardware.joints[0].parameters = {
      {"model", model.name}, {"can_id", "1"}, {"can_timeout_ticks", "4000"},
      {"kp", "20"}, {"kd", "0.5"}, {"command_velocity_max", "2"}};
    const auto joint = rs::parse_driver_configuration(hardware).joints.front();
    EXPECT_DOUBLE_EQ(joint.limits.position_min, -12.566370614);
    EXPECT_DOUBLE_EQ(joint.limits.position_max, 12.566370614);
    EXPECT_DOUBLE_EQ(joint.limits.velocity_min, -model.velocity);
    EXPECT_DOUBLE_EQ(joint.limits.velocity_max, model.velocity);
    EXPECT_DOUBLE_EQ(joint.limits.effort_min, -model.effort);
    EXPECT_DOUBLE_EQ(joint.limits.effort_max, model.effort);
    EXPECT_DOUBLE_EQ(joint.limits.effort_wire_min, -model.effort);
    EXPECT_DOUBLE_EQ(joint.limits.effort_wire_max, model.effort);
    EXPECT_DOUBLE_EQ(joint.limits.kp_max, model.kp);
    EXPECT_DOUBLE_EQ(joint.limits.kd_max, model.kd);
    EXPECT_DOUBLE_EQ(joint.command_limits.velocity_max, 2);
  }
}

TEST(DriverConfig, RejectsUnknownModelAndConflictingOverrides)
{
  auto hardware = valid_hardware_info();
  hardware.joints[0].parameters["model"] = "RS99";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
  hardware.joints[0].parameters["model"] = "EL05";
  EXPECT_NO_THROW(rs::parse_driver_configuration(hardware));
  for (const auto * key : {
      "position_min", "position_max", "velocity_min", "velocity_max", "effort_min",
      "effort_max", "effort_wire_min", "effort_wire_max", "kp_max", "kd_max"})
  {
    for (const auto * value : {"12345", "nan", "50junk"}) {
      auto conflicting = hardware;
      conflicting.joints[0].parameters[key] = value;
      EXPECT_THROW(rs::parse_driver_configuration(conflicting), std::runtime_error) << key;
    }
  }
}

TEST(DriverConfig, CustomProfileRequiresCompleteValidatedLimits)
{
  auto hardware = valid_hardware_info();
  hardware.joints[0].parameters["model"] = "custom";
  EXPECT_NO_THROW(rs::parse_driver_configuration(hardware));
  hardware.joints[0].parameters.erase("velocity_max");
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
}

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

TEST(DriverConfig, UsesAndParsesTransmitFailureTimeout)
{
  auto hardware = valid_hardware_info();
  EXPECT_EQ(
    rs::parse_driver_configuration(hardware).settings.transmit_failure_timeout.count(), 1000);

  hardware.hardware_parameters["transmit_failure_timeout_ms"] = "250";
  EXPECT_EQ(
    rs::parse_driver_configuration(hardware).settings.transmit_failure_timeout.count(), 250);
}

TEST(DriverConfig, RejectsNonPositiveTransmitFailureTimeout)
{
  for (const auto * value : {"0", "-1"}) {
    auto hardware = valid_hardware_info();
    hardware.hardware_parameters["transmit_failure_timeout_ms"] = value;
    EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error) << value;
  }
}
