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

TEST(DriverConfig, StrictlyParsesAllJointNumbersWithContext)
{
  for (const auto * key : {
      "position_min", "position_max", "velocity_min", "velocity_max", "effort_min",
      "effort_max", "effort_wire_min", "effort_wire_max", "kp_max", "kd_max", "kp", "kd",
      "direction", "gear_ratio", "position_offset", "command_position_min",
      "command_position_max", "command_velocity_min", "command_velocity_max",
      "command_effort_min", "command_effort_max"})
  {
    for (const auto * value : {"20oops", "", "   ", "1 2", "nan", "inf", "-inf", "1e9999"}) {
      auto hardware = valid_hardware_info();
      hardware.joints[0].parameters[key] = value;
      SCOPED_TRACE(::testing::Message() << key << "=" << value);
      try {
        rs::parse_driver_configuration(hardware);
        FAIL() << "accepted malformed number";
      } catch (const std::runtime_error & error) {
        const std::string message = error.what();
        EXPECT_NE(message.find(key), std::string::npos);
        EXPECT_NE(message.find("joint_1"), std::string::npos);
        EXPECT_NE(message.find("test_system"), std::string::npos);
      }
    }
  }
}

TEST(DriverConfig, StrictlyParsesHardwareIntegersWithContext)
{
  for (const auto * key : {
      "host_can_id", "can_rx_qos_depth", "feedback_timeout_ms", "transmit_failure_timeout_ms",
      "run_mode_recovery_timeout_ms", "run_mode_recovery_retry_interval_ms",
      "shutdown_stop_repetitions", "shutdown_stop_interval_ms", "shutdown_confirmation_timeout_ms",
      "startup_connection_timeout_ms", "startup_confirmation_timeout_ms", "startup_retries"})
  {
    for (const auto * value : {
        "1oops", "-1", "-0", "1.5", "", " ", "1 2", "nan", "inf", "18446744073709551616"})
    {
      auto hardware = valid_hardware_info();
      hardware.hardware_parameters[key] = value;
      SCOPED_TRACE(::testing::Message() << key << "=" << value);
      try {
        rs::parse_driver_configuration(hardware);
        FAIL() << "accepted malformed integer";
      } catch (const std::runtime_error & error) {
        EXPECT_NE(std::string(error.what()).find(key), std::string::npos);
        EXPECT_NE(std::string(error.what()).find("test_system"), std::string::npos);
      }
    }
  }
}

TEST(DriverConfig, StrictlyParsesJointIntegers)
{
  for (const auto * key : {"can_id", "can_timeout_ticks"}) {
    for (const auto * value : {"1oops", "-1", "-0", "1.5", "0x1g", "", "4294967296"}) {
      auto hardware = valid_hardware_info();
      hardware.joints[0].parameters[key] = value;
      try {
        rs::parse_driver_configuration(hardware);
        FAIL() << key << "=" << value;
      } catch (const std::runtime_error & error) {
        EXPECT_NE(std::string(error.what()).find(key), std::string::npos);
        EXPECT_NE(std::string(error.what()).find("joint_1"), std::string::npos);
      }
    }
  }
}

TEST(DriverConfig, AcceptsWhitespaceSignsAndDocumentedBases)
{
  auto hardware = valid_hardware_info();
  hardware.hardware_parameters["host_can_id"] = " \t+0xFD\n";
  hardware.hardware_parameters["startup_retries"] = " 010 ";
  hardware.joints[0].parameters["can_id"] = " 0377 ";
  hardware.joints[0].parameters["can_timeout_ticks"] = " 0xffffffff ";
  hardware.joints[0].parameters["kp"] = " +3e1 ";
  hardware.joints[0].parameters["position_offset"] = " -0.5 ";
  const auto config = rs::parse_driver_configuration(hardware);
  EXPECT_EQ(config.settings.host_id, 253);
  EXPECT_EQ(config.settings.startup_retries, 10);
  EXPECT_EQ(config.joints[0].can_id, 255);
  EXPECT_EQ(config.joints[0].can_timeout_ticks, std::numeric_limits<uint32_t>::max());
  EXPECT_DOUBLE_EQ(config.joints[0].kp, 30.0);
  EXPECT_DOUBLE_EQ(config.joints[0].position_offset, -0.5);
}

TEST(DriverConfig, EnforcesQosAndIntegerBounds)
{
  auto hardware = valid_hardware_info();
  for (const auto * value : {"0", "4097", "18446744073709551615"}) {
    hardware.hardware_parameters["can_rx_qos_depth"] = value;
    EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
  }
  for (const auto * value : {"1", "4096"}) {
    hardware.hardware_parameters["can_rx_qos_depth"] = value;
    EXPECT_NO_THROW(rs::parse_driver_configuration(hardware));
  }
  hardware.hardware_parameters["startup_retries"] = "2147483648";
  EXPECT_THROW(rs::parse_driver_configuration(hardware), std::runtime_error);
  hardware.hardware_parameters["startup_retries"] = "2147483647";
  hardware.hardware_parameters["shutdown_stop_interval_ms"] = "0";
  hardware.hardware_parameters["shutdown_confirmation_timeout_ms"] = "0";
  hardware.hardware_parameters["host_can_id"] = "0";
  EXPECT_NO_THROW(rs::parse_driver_configuration(hardware));
}

TEST(DriverConfig, RejectsNeutralRangesExcludingZero)
{
  for (const std::string prefix : {
      "command_velocity", "command_effort", "velocity", "effort", "effort_wire"})
  {
    for (const bool positive : {true, false}) {
      auto hardware = valid_hardware_info();
      hardware.joints[0].parameters[prefix + "_min"] = positive ? "1" : "-3";
      hardware.joints[0].parameters[prefix + "_max"] = positive ? "3" : "-1";
      SCOPED_TRACE(prefix);
      try {
        rs::parse_driver_configuration(hardware);
        FAIL() << "accepted a range excluding zero";
      } catch (const std::runtime_error & error) {
        EXPECT_NE(std::string(error.what()).find("joint_1"), std::string::npos);
        EXPECT_NE(std::string(error.what()).find("include zero"), std::string::npos);
      }
    }
  }
}

TEST(DriverConfig, AllowsZeroAtOperationalBoundaryAndNonzeroPositionRange)
{
  for (const bool positive : {true, false}) {
    auto hardware = valid_hardware_info();
    auto & params = hardware.joints[0].parameters;
    params["command_position_min"] = "1";
    params["command_position_max"] = "2";
    for (const std::string prefix : {"command_velocity", "command_effort"}) {
      params[prefix + "_min"] = positive ? "0" : "-3";
      params[prefix + "_max"] = positive ? "3" : "0";
    }
    EXPECT_NO_THROW(rs::parse_driver_configuration(hardware));
  }
}

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
