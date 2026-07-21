#include "robstride_ros2/robstride_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "robstride_ros2/command_mode.hpp"

namespace robstride_ros2
{
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

rclcpp::Logger driver_logger()
{
  return rclcpp::get_logger("RobStrideSystem");
}
}  // namespace

RobStrideSystem::~RobStrideSystem()
{
  try {
    if (active_) {disable_all();}
    stop_transport();
  } catch (...) {}
}

bool RobStrideSystem::parse_bool(const std::string & value)
{
  if (value == "true" || value == "1") {return true;}
  if (value == "false" || value == "0") {return false;}
  throw std::runtime_error("boolean parameter must be true, false, 1, or 0; got '" + value + "'");
}

#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
hardware_interface::CallbackReturn RobStrideSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return initialize_from_info(params.hardware_info);
}
#else
hardware_interface::CallbackReturn RobStrideSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return initialize_from_info(info);
}
#endif

hardware_interface::CallbackReturn RobStrideSystem::initialize_from_info(
  const hardware_interface::HardwareInfo & info)
{
  try {
    const int host_can_id =
      std::stoi(hardware_parameter_or(info, "host_can_id", "253"), nullptr, 0);
    if (host_can_id < 0 || host_can_id > 255) {
      throw std::runtime_error("host_can_id must be 0..255");
    }
    settings_.host_id = static_cast<uint8_t>(host_can_id);
    settings_.transport.node_name = info.name + "_can_transport";
    settings_.transport.transmit_topic =
      hardware_parameter_or(info, "can_tx_topic", "to_can_bus");
    settings_.transport.receive_topic =
      hardware_parameter_or(info, "can_rx_topic", "from_can_bus");

    const auto receive_qos_parameter = info.hardware_parameters.find("can_rx_qos_depth");
    const auto legacy_qos_parameter = info.hardware_parameters.find("can_qos_depth");
    const std::string receive_qos_depth = receive_qos_parameter != info.hardware_parameters.end() ?
      receive_qos_parameter->second :
      (legacy_qos_parameter != info.hardware_parameters.end() ? legacy_qos_parameter->second : "32");
    settings_.transport.receive_qos_depth = static_cast<size_t>(std::stoul(receive_qos_depth));
    if (receive_qos_parameter == info.hardware_parameters.end() &&
      legacy_qos_parameter != info.hardware_parameters.end())
    {
      RCLCPP_WARN(
        driver_logger(),
        "Hardware parameter 'can_qos_depth' is deprecated; use 'can_rx_qos_depth'");
    }

    settings_.feedback_timeout = std::chrono::milliseconds(std::stoi(
          hardware_parameter_or(info, "feedback_timeout_ms", "3000")));
    settings_.fail_on_feedback_timeout = parse_bool(
      hardware_parameter_or(info, "fail_on_feedback_timeout", "true"));
    settings_.run_mode_recovery_timeout = std::chrono::milliseconds(std::stoi(
          hardware_parameter_or(info, "run_mode_recovery_timeout_ms", "500")));
    settings_.run_mode_recovery_retry_interval = std::chrono::milliseconds(std::stoi(
          hardware_parameter_or(info, "run_mode_recovery_retry_interval_ms", "100")));
    settings_.clear_faults_on_activate = parse_bool(
      hardware_parameter_or(info, "clear_faults_on_activate", "true"));
    settings_.set_zero_on_activate = parse_bool(
      hardware_parameter_or(info, "set_zero_on_activate", "false"));
    settings_.shutdown_stop_repetitions = std::stoi(
      hardware_parameter_or(info, "shutdown_stop_repetitions", "3"));
    settings_.shutdown_stop_interval = std::chrono::milliseconds(std::stoi(
          hardware_parameter_or(info, "shutdown_stop_interval_ms", "20")));
    settings_.shutdown_confirmation_timeout = std::chrono::milliseconds(std::stoi(
          hardware_parameter_or(info, "shutdown_confirmation_timeout_ms", "300")));
    settings_.startup_connection_timeout = std::chrono::milliseconds(std::stoi(
          hardware_parameter_or(info, "startup_connection_timeout_ms", "3000")));
    settings_.startup_confirmation_timeout = std::chrono::milliseconds(std::stoi(
          hardware_parameter_or(info, "startup_confirmation_timeout_ms", "500")));
    settings_.startup_retries =
      std::stoi(hardware_parameter_or(info, "startup_retries", "3"));

    if (settings_.transport.receive_qos_depth == 0 || settings_.feedback_timeout.count() <= 0 ||
      settings_.run_mode_recovery_timeout.count() <= 0 ||
      settings_.run_mode_recovery_retry_interval.count() <= 0 ||
      settings_.run_mode_recovery_retry_interval > settings_.run_mode_recovery_timeout)
    {
      throw std::runtime_error(
              "CAN QoS depth, feedback timeout, and Run-mode recovery "
              "timings must be positive; recovery retry interval must not exceed its timeout");
    }
    if (settings_.shutdown_stop_repetitions <= 0 ||
      settings_.shutdown_stop_interval.count() < 0 ||
      settings_.shutdown_confirmation_timeout.count() < 0 ||
      settings_.startup_connection_timeout.count() <= 0 ||
      settings_.startup_confirmation_timeout.count() <= 0 || settings_.startup_retries <= 0)
    {
      throw std::runtime_error("invalid startup or shutdown timing parameters");
    }

    joints_.clear();
    can_id_to_joint_.clear();
    if (info.joints.empty()) {throw std::runtime_error("at least one joint is required");}
    std::set<uint8_t> configured_can_ids;
    joints_.reserve(info.joints.size());
    for (const auto & joint_info : info.joints) {
      Joint joint;
      if (!parse_joint(joint_info, joint)) {return hardware_interface::CallbackReturn::ERROR;}
      if (!configured_can_ids.insert(joint.can_id).second) {
        throw std::runtime_error("duplicate can_id " + std::to_string(joint.can_id));
      }
      can_id_to_joint_[joint.can_id] = joints_.size();
      joints_.push_back(joint);
    }
    settings_.transport.motor_count = joints_.size();
    runtime_events_.clear();
    runtime_events_.reserve(joints_.size());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(driver_logger(), "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RobStrideSystem::parse_joint(const hardware_interface::ComponentInfo & info, Joint & joint)
{
  try {
    joint.name = info.name;
    const double unknown_state_value = std::numeric_limits<double>::quiet_NaN();
    joint.state = StateValues{
      unknown_state_value, unknown_state_value, unknown_state_value, unknown_state_value, 0.0};
    joint.feedback = joint.state;
    joint.command.position = std::numeric_limits<double>::quiet_NaN();
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
    joint.can_timeout_ticks = static_cast<uint32_t>(
      std::stoul(info.parameters.at("can_timeout_ticks"), nullptr, 0));
    if (joint.can_timeout_ticks == 0) {
      throw std::runtime_error("can_timeout_ticks must be nonzero for fail-safe shutdown");
    }
    const auto direction_parameter = info.parameters.find("direction");
    joint.direction = direction_parameter == info.parameters.end() ?
      1.0 : std::stod(direction_parameter->second);
    const auto gear_ratio_parameter = info.parameters.find("gear_ratio");
    joint.gear_ratio = gear_ratio_parameter == info.parameters.end() ?
      1.0 : std::stod(gear_ratio_parameter->second);
    const auto position_offset_parameter = info.parameters.find("position_offset");
    joint.position_offset = position_offset_parameter == info.parameters.end() ?
      0.0 : std::stod(position_offset_parameter->second);
    if (joint.direction != 1.0 && joint.direction != -1.0) {
      throw std::runtime_error("direction must be 1 or -1");
    }
    if (!(joint.gear_ratio > 0.0) || !std::isfinite(joint.gear_ratio)) {
      throw std::runtime_error("gear_ratio must be finite and positive");
    }
    if (!(joint.limits.position_min < joint.limits.position_max) ||
      !(joint.limits.velocity_min < joint.limits.velocity_max) ||
      !(joint.limits.effort_min < joint.limits.effort_max) || joint.limits.kp_max <= 0.0 ||
      !(joint.limits.effort_wire_min < joint.limits.effort_wire_max) ||
      joint.limits.kd_max <= 0.0 || joint.kp < 0.0 || joint.kp > joint.limits.kp_max ||
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
    std::set<std::string> actual_states;
    for (const auto & interface : info.state_interfaces) {actual_states.insert(interface.name);}
    const std::set<std::string> allowed_states{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT, "temperature", "fault"};
    if (!std::includes(
        actual_states.begin(), actual_states.end(), required_states.begin(), required_states.end()) ||
      !std::includes(
        allowed_states.begin(), allowed_states.end(), actual_states.begin(), actual_states.end()) ||
      actual_states.size() != info.state_interfaces.size())
    {
      throw std::runtime_error("state interfaces must include position, velocity and effort");
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      driver_logger(), "Joint '%s': %s", info.name.c_str(), error.what());
    return false;
  }
  return true;
}

std::vector<hardware_interface::StateInterface> RobStrideSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.state.position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.state.effort);
    const auto & declared = info_.joints[&joint - joints_.data()].state_interfaces;
    for (const auto & item : declared) {
      if (item.name == "temperature") {
        interfaces.emplace_back(joint.name, item.name, &joint.state.temperature);
      } else if (item.name == "fault") {
        interfaces.emplace_back(joint.name, item.name, &joint.state.fault);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobStrideSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : joints_) {
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.command.position);
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.command.effort);
  }
  return interfaces;
}

hardware_interface::CallbackReturn RobStrideSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    transport_ = std::make_unique<CanTransport>(
      settings_.transport,
      [this](can_msgs::msg::Frame::ConstSharedPtr frame) {receive_frame(std::move(frame));});
    transport_->start();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(driver_logger(), "CAN topic setup failed: %s", error.what());
    stop_transport();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!transport_ || !transport_->wait_for_endpoints(settings_.startup_connection_timeout)) {
    RCLCPP_ERROR(
      driver_logger(),
      "Timed out waiting for ros2_socketcan endpoints on '%s' and '%s'",
      settings_.transport.transmit_topic.c_str(), settings_.transport.receive_topic.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_) {
    if (settings_.clear_faults_on_activate) {
      transport_->send_transaction(make_stop(joint.can_id, settings_.host_id, true));
    }
    if (!write_and_confirm_parameter(joint, kIndexCanTimeout, joint.can_timeout_ticks) ||
      !write_and_confirm_parameter(joint, kIndexRunMode, 0))
    {
      disable_all();
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (settings_.set_zero_on_activate) {
      transport_->send_transaction(make_set_zero(joint.can_id, settings_.host_id));
    }
  }

  if (!enable_and_confirm_all()) {
    disable_all();
    return hardware_interface::CallbackReturn::ERROR;
  }
  {
    // Controllers read exported state variables without holding state_mutex_.
    // Seed them before activation, then update them only from read().
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t joint_index = 0; joint_index < joints_.size(); ++joint_index) {
      auto & joint = joints_[joint_index];
      joint.state = joint.feedback;
      joint.run_mode_recovery = RunModeRecoveryState{};
    }
  }
  transport_->enable_active_commands();
  activated_at_ = std::chrono::steady_clock::now();
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  disable_all();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  disable_all();
  stop_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_error(const rclcpp_lifecycle::State &)
{
  disable_all();
  stop_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobStrideSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  const auto now = std::chrono::steady_clock::now();
  bool read_failed = false;
  runtime_events_.clear();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t joint_index = 0; joint_index < joints_.size(); ++joint_index) {
      auto & joint = joints_[joint_index];
      if (joint.feedback_status.received) {
        joint.state = joint.feedback;
      }

      if (settings_.fail_on_feedback_timeout) {
        const auto reference =
          joint.feedback_status.received ? joint.feedback_status.timestamp : activated_at_;
        if (now - reference > settings_.feedback_timeout) {
          runtime_events_.push_back(
            RuntimeEvent{RuntimeEventKind::feedback_timeout, joint_index});
          read_failed = true;
          break;
        }
      }

      if (!joint.feedback_status.received) {continue;}
      auto & recovery = joint.run_mode_recovery;
      const int attempts_before_update = recovery.attempts;
      const auto recovery_action = update_run_mode_recovery(
        recovery, joint.feedback_status.mode, now, settings_.run_mode_recovery_timeout,
        settings_.run_mode_recovery_retry_interval);
      if (recovery_action == RunModeRecoveryAction::recovered) {
        transport_->complete_recovery(joint_index);
        runtime_events_.push_back(
          RuntimeEvent{
            RuntimeEventKind::recovered, joint_index, kMotorModeRun, attempts_before_update});
      } else if (recovery_action == RunModeRecoveryAction::failed) {
        runtime_events_.push_back(
          RuntimeEvent{
            RuntimeEventKind::recovery_failed, joint_index, recovery.detected_mode,
            recovery.attempts});
        read_failed = true;
        break;
      } else if (recovery_action == RunModeRecoveryAction::send_enable) {
        if (recovery.attempts == 1) {
          runtime_events_.push_back(
            RuntimeEvent{
              RuntimeEventKind::recovery_started, joint_index, recovery.detected_mode,
              recovery.attempts});
        }
        transport_->queue_recovery_frame(
          joint_index, make_enable(joint.can_id, settings_.host_id));
      }
    }
  }

  for (const auto & event : runtime_events_) {
    const auto & joint = joints_[event.joint_index];
    if (event.kind == RuntimeEventKind::feedback_timeout) {
      RCLCPP_ERROR_THROTTLE(
        driver_logger(), *log_clock_, 1000, "Feedback timeout for joint '%s'",
        joint.name.c_str());
    } else if (event.kind == RuntimeEventKind::recovery_started) {
      RCLCPP_WARN(
        driver_logger(),
        "Joint '%s' left Run mode while hardware is active (mode=%u); attempting recovery",
        joint.name.c_str(), static_cast<unsigned int>(event.mode));
    } else if (event.kind == RuntimeEventKind::recovered) {
      RCLCPP_INFO(
        driver_logger(), "Joint '%s' recovered to Run mode after %d enable attempt(s)",
        joint.name.c_str(), event.attempts);
    } else if (event.kind == RuntimeEventKind::recovery_failed) {
      RCLCPP_ERROR(
        driver_logger(),
        "Joint '%s' did not return to Run mode within %lld ms after %d enable attempt(s)",
        joint.name.c_str(),
        static_cast<long long>(settings_.run_mode_recovery_timeout.count()), event.attempts);
    }
  }
  if (read_failed) {return hardware_interface::return_type::ERROR;}
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!active_) {return hardware_interface::return_type::OK;}
  for (size_t joint_index = 0; joint_index < joints_.size(); ++joint_index) {
    const auto & joint = joints_[joint_index];
    const double joint_position = joint.claimed.position && std::isfinite(joint.command.position) ?
      joint.command.position : joint.state.position;
    const double motor_position = std::isfinite(joint_position) ?
      joint.direction * (joint_position - joint.position_offset) * joint.gear_ratio : 0.0;
    const double motor_velocity = joint.claimed.velocity && std::isfinite(joint.command.velocity) ?
      joint.direction * joint.command.velocity * joint.gear_ratio : 0.0;
    const double motor_effort = joint.claimed.effort && std::isfinite(joint.command.effort) ?
      joint.direction * joint.command.effort : 0.0;
    const double kp = joint.claimed.position ? joint.kp : 0.0;
    const double kd = (joint.claimed.position || joint.claimed.velocity) ? joint.kd : 0.0;
    transport_->queue_motion_frame(
      joint_index, make_motion_command(
        joint.can_id, joint.limits, motor_position, motor_velocity, motor_effort, kp, kd));
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<CommandModeState> current_states;
  current_states.reserve(joints_.size());
  for (const auto & joint : joints_) {
    current_states.push_back(CommandModeState{
      joint.name, joint.claimed.position, joint.claimed.velocity, joint.claimed.effort});
  }

  std::string validation_error;
  if (!validate_command_mode_switch(current_states, start, stop, &validation_error)) {
    RCLCPP_ERROR(
      driver_logger(), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & key : start) {
    for (const auto & joint : joints_) {
      if (key == joint.name + "/position" && !joint.feedback_status.received) {
        RCLCPP_ERROR(
          driver_logger(),
          "Cannot start position command interface for '%s' before first feedback",
          joint.name.c_str());
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto set_claimed = [this](const std::string & key, bool value) {
      for (auto & joint : joints_) {
        if (key == joint.name + "/position") {
          joint.claimed.position = value;
          if (value) {joint.command.position = joint.state.position;}
        } else if (key == joint.name + "/velocity") {
          joint.claimed.velocity = value;
          if (value) {joint.command.velocity = 0.0;}
        } else if (key == joint.name + "/effort") {
          joint.claimed.effort = value;
          if (value) {joint.command.effort = 0.0;}
        }
      }
    };
  for (const auto & key : stop) {set_claimed(key, false);}
  for (const auto & key : start) {set_claimed(key, true);}
  return hardware_interface::return_type::OK;
}

void RobStrideSystem::receive_frame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  if (msg->is_error || msg->is_rtr) {return;}
  const uint16_t data_area = static_cast<uint16_t>((msg->id >> 8) & 0xffff);
  const uint8_t motor_id = static_cast<uint8_t>(data_area & 0xff);
  const auto joint_entry = can_id_to_joint_.find(motor_id);
  if (joint_entry == can_id_to_joint_.end()) {return;}

  const auto parameter = decode_parameter_response(
    msg->id, msg->data, msg->dlc, msg->is_extended, settings_.host_id);
  if (parameter) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto & joint = joints_[joint_entry->second];
    joint.parameter_status.received = true;
    joint.parameter_status.index = parameter->index;
    joint.parameter_status.value = parameter->value;
    joint.parameter_status.timestamp = std::chrono::steady_clock::now();
    feedback_condition_.notify_all();
    return;
  }

  const auto decoded = decode_feedback(
    msg->id, msg->data, msg->dlc, msg->is_extended, settings_.host_id,
    joints_[joint_entry->second].limits);
  if (!decoded) {return;}
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto & joint = joints_[joint_entry->second];
  joint.feedback.position =
    joint.direction * (decoded->position / joint.gear_ratio) + joint.position_offset;
  joint.feedback.velocity = joint.direction * (decoded->velocity / joint.gear_ratio);
  joint.feedback.effort = joint.direction * decoded->effort;
  joint.feedback.temperature = decoded->temperature;
  joint.feedback.fault = decoded->fault_flags;
  joint.feedback_status.mode = decoded->mode;
  joint.feedback_status.received = true;
  joint.feedback_status.timestamp = std::chrono::steady_clock::now();
  feedback_condition_.notify_all();
}

bool RobStrideSystem::write_and_confirm_parameter(
  Joint & joint, uint16_t index, uint32_t value)
{
  for (int attempt = 1; attempt <= settings_.startup_retries; ++attempt) {
    const auto requested_at = std::chrono::steady_clock::now();
    if (index == kIndexRunMode) {
      transport_->send_transaction(
        make_write_u8(joint.can_id, settings_.host_id, index, static_cast<uint8_t>(value)));
    } else {
      transport_->send_transaction(
        make_write_u32(joint.can_id, settings_.host_id, index, value));
    }
    transport_->send_transaction(
      make_read_parameter(joint.can_id, settings_.host_id, index));

    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, settings_.startup_confirmation_timeout,
      [&joint, index, value, requested_at]() {
        return joint.parameter_status.received &&
               joint.parameter_status.timestamp >= requested_at &&
               joint.parameter_status.index == index && joint.parameter_status.value == value;
      });
    if (confirmed) {return true;}
    RCLCPP_WARN(
      driver_logger(),
      "Joint '%s': parameter 0x%04X confirmation attempt %d/%d failed",
      joint.name.c_str(), index, attempt, settings_.startup_retries);
  }
  RCLCPP_ERROR(
    driver_logger(), "Joint '%s': parameter 0x%04X could not be confirmed",
    joint.name.c_str(), index);
  return false;
}

bool RobStrideSystem::enable_and_confirm_all()
{
  for (int attempt = 1; attempt <= settings_.startup_retries; ++attempt) {
    const auto requested_at = std::chrono::steady_clock::now();
    for (const auto & joint : joints_) {
      transport_->send_transaction(make_enable(joint.can_id, settings_.host_id));
      transport_->send_transaction(make_motion_command(
          joint.can_id, joint.limits, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, settings_.startup_confirmation_timeout,
      [this, requested_at]() {
        return std::all_of(joints_.begin(), joints_.end(), [requested_at](const Joint & joint) {
          return joint.feedback_status.received &&
                 joint.feedback_status.timestamp >= requested_at &&
                 joint.feedback_status.mode == kMotorModeRun;
        });
      });
    if (confirmed) {return true;}
    RCLCPP_WARN(
      driver_logger(), "Motor enable confirmation attempt %d/%d failed",
      attempt, settings_.startup_retries);
  }
  RCLCPP_ERROR(driver_logger(), "Not all motors entered Run mode");
  return false;
}

void RobStrideSystem::disable_all()
{
  if (!transport_) {
    active_ = false;
    return;
  }
  active_ = false;
  transport_->disable_active_commands();
  const auto stop_started = std::chrono::steady_clock::now();
  bool stop_frames_acknowledged = false;
  for (int attempt = 0; attempt < settings_.shutdown_stop_repetitions; ++attempt) {
    for (const auto & joint : joints_) {
      transport_->send_transaction(
        make_motion_command(joint.can_id, joint.limits, 0.0, 0.0, 0.0, 0.0, 0.0));
      transport_->send_transaction(make_stop(joint.can_id, settings_.host_id));
    }
    stop_frames_acknowledged =
      transport_->wait_for_transaction_acknowledgements(std::chrono::milliseconds(50));
    if (attempt + 1 < settings_.shutdown_stop_repetitions &&
      settings_.shutdown_stop_interval.count() > 0)
    {
      std::this_thread::sleep_for(settings_.shutdown_stop_interval);
    }
  }
  if (!stop_frames_acknowledged) {
    RCLCPP_WARN(
      driver_logger(),
      "Stop frames were not acknowledged by DDS; transport shutdown will still drain its local "
      "transaction queue and the motor watchdog remains the final fallback");
  }

  if (settings_.shutdown_confirmation_timeout.count() > 0) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, settings_.shutdown_confirmation_timeout,
      [this, stop_started]() {
        return std::all_of(joints_.begin(), joints_.end(), [stop_started](const Joint & joint) {
          return joint.feedback_status.received &&
                 joint.feedback_status.timestamp >= stop_started &&
                 joint.feedback_status.mode == kMotorModeReset;
        });
      });
    if (!confirmed) {
      RCLCPP_ERROR(
        driver_logger(),
        "Motor stop was not confirmed on CAN; motor watchdog must force reset after timeout");
    }
  }
}

void RobStrideSystem::stop_transport()
{
  active_ = false;
  if (transport_) {transport_->stop();}
  transport_.reset();
}

}  // namespace robstride_ros2

PLUGINLIB_EXPORT_CLASS(robstride_ros2::RobStrideSystem, hardware_interface::SystemInterface)
