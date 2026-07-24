#include "robstride_ros2_control/driver_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace robstride_ros2_control
{
using robstride_driver::DriverConfiguration;
using robstride_driver::JointData;
using robstride_driver::Limits;
namespace
{
double required_number(
  const std::unordered_map<std::string, std::string> & parameters, const char * key)
{
  const auto it = parameters.find(key);
  if (it == parameters.end()) {
    throw std::runtime_error(std::string("missing parameter '") + key + "'");
  }
  return std::stod(it->second);
}

double number_or_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const char * preferred_key, const char * default_key)
{
  return parameters.count(preferred_key) ?
         required_number(parameters, preferred_key) : required_number(parameters, default_key);
}

std::string hardware_parameter_or(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & default_value)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? default_value : it->second;
}

bool parse_bool(const std::string & value)
{
  if (value == "true" || value == "1") {return true;}
  if (value == "false" || value == "0") {return false;}
  throw std::runtime_error("boolean parameter must be true, false, 1, or 0; got '" + value + "'");
}

bool is_blank(const std::string & value)
{
  return value.find_first_not_of(" \t\r\n") == std::string::npos;
}

JointData parse_joint(const hardware_interface::ComponentInfo & info)
{
  try {
    JointData joint;
    joint.name = info.name;
    joint.feedback = joint.state;
    const int motor_can_id = std::stoi(info.parameters.at("can_id"), nullptr, 0);
    if (motor_can_id < 1 || motor_can_id > 255) {
      throw std::runtime_error("can_id must be 1..255");
    }
    joint.can_id = static_cast<uint8_t>(motor_can_id);
    joint.limits = Limits{
      required_number(info.parameters, "position_min"),
      required_number(info.parameters, "position_max"),
      required_number(info.parameters, "velocity_min"),
      required_number(info.parameters, "velocity_max"),
      required_number(info.parameters, "effort_min"),
      required_number(info.parameters, "effort_max"),
      number_or_parameter(info.parameters, "effort_wire_min", "effort_min"),
      number_or_parameter(info.parameters, "effort_wire_max", "effort_max"),
      required_number(info.parameters, "kp_max"), required_number(info.parameters, "kd_max")};
    joint.kp = required_number(info.parameters, "kp");
    joint.kd = required_number(info.parameters, "kd");
    const auto & watchdog_text = info.parameters.at("can_timeout_ticks");
    size_t watchdog_characters = 0;
    const auto watchdog_value = std::stoull(watchdog_text, &watchdog_characters, 0);
    if (watchdog_characters != watchdog_text.size() || watchdog_value == 0 ||
      watchdog_value > std::numeric_limits<uint32_t>::max())
    {
      throw std::runtime_error("can_timeout_ticks must be a nonzero uint32 value");
    }
    joint.can_timeout_ticks = static_cast<uint32_t>(watchdog_value);
    const auto direction = info.parameters.find("direction");
    joint.direction = direction == info.parameters.end() ? 1.0 : std::stod(direction->second);
    const auto gear_ratio = info.parameters.find("gear_ratio");
    joint.gear_ratio =
      gear_ratio == info.parameters.end() ? 1.0 : std::stod(gear_ratio->second);
    const auto position_offset = info.parameters.find("position_offset");
    joint.position_offset =
      position_offset == info.parameters.end() ? 0.0 : std::stod(position_offset->second);
    if (joint.direction != 1.0 && joint.direction != -1.0) {
      throw std::runtime_error("direction must be 1 or -1");
    }
    if (!(joint.gear_ratio > 0.0) || !std::isfinite(joint.gear_ratio)) {
      throw std::runtime_error("gear_ratio must be finite and positive");
    }
    if (!std::isfinite(joint.position_offset)) {
      throw std::runtime_error("position_offset must be finite");
    }
    if (!std::isfinite(joint.limits.position_min) ||
      !std::isfinite(joint.limits.position_max) ||
      !std::isfinite(joint.limits.velocity_min) ||
      !std::isfinite(joint.limits.velocity_max) ||
      !std::isfinite(joint.limits.effort_min) ||
      !std::isfinite(joint.limits.effort_max) ||
      !std::isfinite(joint.limits.effort_wire_min) ||
      !std::isfinite(joint.limits.effort_wire_max) ||
      !std::isfinite(joint.limits.kp_max) || !std::isfinite(joint.limits.kd_max) ||
      !std::isfinite(joint.kp) || !std::isfinite(joint.kd) ||
      !(joint.limits.position_min < joint.limits.position_max) ||
      !(joint.limits.velocity_min < joint.limits.velocity_max) ||
      !(joint.limits.effort_min < joint.limits.effort_max) ||
      !(joint.limits.effort_wire_min < joint.limits.effort_wire_max) ||
      joint.limits.kp_max <= 0.0 || joint.limits.kd_max <= 0.0 ||
      joint.kp < 0.0 || joint.kp > joint.limits.kp_max ||
      joint.kd < 0.0 || joint.kd > joint.limits.kd_max)
    {
      throw std::runtime_error("invalid limits or gains");
    }

    const std::set<std::string> expected_commands{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT};
    std::set<std::string> actual_commands;
    for (const auto & interface : info.command_interfaces) {actual_commands.insert(interface.name);}
    if (actual_commands != expected_commands || info.command_interfaces.size() != 3) {
      throw std::runtime_error("command interfaces must be position, velocity and effort");
    }

    const std::set<std::string> required_states{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT};
    const std::set<std::string> allowed_states{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT, "temperature", "fault"};
    std::set<std::string> actual_states;
    for (const auto & interface : info.state_interfaces) {actual_states.insert(interface.name);}
    if (!std::includes(
        actual_states.begin(), actual_states.end(), required_states.begin(), required_states.end()) ||
      !std::includes(
        allowed_states.begin(), allowed_states.end(), actual_states.begin(), actual_states.end()) ||
      actual_states.size() != info.state_interfaces.size())
    {
      throw std::runtime_error("state interfaces must include position, velocity and effort");
    }
    joint.exports_temperature = actual_states.count("temperature") != 0;
    joint.exports_fault = actual_states.count("fault") != 0;
    return joint;
  } catch (const std::exception & error) {
    throw std::runtime_error("Joint '" + info.name + "': " + error.what());
  }
}
}  // namespace

DriverConfiguration parse_driver_configuration(const hardware_interface::HardwareInfo & info)
{
  DriverConfiguration configuration;
  auto & settings = configuration.settings;
  const int host_can_id = std::stoi(hardware_parameter_or(info, "host_can_id", "253"), nullptr, 0);
  if (host_can_id < 0 || host_can_id > 255) {
    throw std::runtime_error("host_can_id must be 0..255");
  }
  settings.host_id = static_cast<uint8_t>(host_can_id);
  settings.transport.node_name = info.name + "_can_transport";
  settings.transport.transmit_topic = hardware_parameter_or(info, "can_tx_topic", "to_can_bus");
  settings.transport.receive_topic = hardware_parameter_or(info, "can_rx_topic", "from_can_bus");
  if (is_blank(settings.transport.transmit_topic) || is_blank(settings.transport.receive_topic)) {
    throw std::runtime_error("CAN transmit and receive topic names must not be empty");
  }

  settings.transport.receive_qos_depth = static_cast<size_t>(
    std::stoul(hardware_parameter_or(info, "can_rx_qos_depth", "32")));

  settings.feedback_timeout = std::chrono::milliseconds(
    std::stoi(hardware_parameter_or(info, "feedback_timeout_ms", "3000")));
  settings.fail_on_feedback_timeout = parse_bool(
    hardware_parameter_or(info, "fail_on_feedback_timeout", "true"));
  settings.recovery_timeout = std::chrono::milliseconds(
    std::stoi(hardware_parameter_or(info, "run_mode_recovery_timeout_ms", "500")));
  settings.recovery_retry_interval = std::chrono::milliseconds(
    std::stoi(hardware_parameter_or(info, "run_mode_recovery_retry_interval_ms", "100")));
  settings.clear_faults_on_start = parse_bool(
    hardware_parameter_or(info, "clear_faults_on_activate", "true"));
  settings.set_zero_on_start = parse_bool(
    hardware_parameter_or(info, "set_zero_on_activate", "false"));
  settings.stop_repetitions =
    std::stoi(hardware_parameter_or(info, "shutdown_stop_repetitions", "3"));
  settings.stop_interval = std::chrono::milliseconds(
    std::stoi(hardware_parameter_or(info, "shutdown_stop_interval_ms", "20")));
  settings.stop_confirmation_timeout = std::chrono::milliseconds(
    std::stoi(hardware_parameter_or(info, "shutdown_confirmation_timeout_ms", "300")));
  settings.connection_timeout = std::chrono::milliseconds(
    std::stoi(hardware_parameter_or(info, "startup_connection_timeout_ms", "3000")));
  settings.startup_confirmation_timeout = std::chrono::milliseconds(
    std::stoi(hardware_parameter_or(info, "startup_confirmation_timeout_ms", "500")));
  settings.startup_retries = std::stoi(hardware_parameter_or(info, "startup_retries", "3"));

  if (settings.transport.receive_qos_depth == 0 || settings.feedback_timeout.count() <= 0 ||
    settings.recovery_timeout.count() <= 0 || settings.recovery_retry_interval.count() <= 0 ||
    settings.recovery_retry_interval > settings.recovery_timeout)
  {
    throw std::runtime_error(
            "CAN QoS depth, feedback timeout, and Run-mode recovery timings must be positive; "
            "recovery retry interval must not exceed its timeout");
  }
  if (settings.stop_repetitions <= 0 || settings.stop_interval.count() < 0 ||
    settings.stop_confirmation_timeout.count() < 0 || settings.connection_timeout.count() <= 0 ||
    settings.startup_confirmation_timeout.count() <= 0 || settings.startup_retries <= 0)
  {
    throw std::runtime_error("invalid startup or shutdown timing parameters");
  }

  if (info.joints.empty()) {throw std::runtime_error("at least one joint is required");}
  std::set<uint8_t> configured_can_ids;
  configuration.joints.reserve(info.joints.size());
  for (const auto & joint_info : info.joints) {
    auto joint = parse_joint(joint_info);
    if (!configured_can_ids.insert(joint.can_id).second) {
      throw std::runtime_error("duplicate can_id " + std::to_string(joint.can_id));
    }
    configuration.joints.push_back(std::move(joint));
  }
  settings.transport.motor_count = configuration.joints.size();
  return configuration;
}

}  // namespace robstride_ros2_control
