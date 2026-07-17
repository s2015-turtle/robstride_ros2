#include "robstride_ros2/robstride_system.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "robstride_ros2/command_mode.hpp"

namespace robstride_ros2
{
namespace
{
double number(const std::unordered_map<std::string, std::string> & parameters, const char * key)
{
  const auto it = parameters.find(key);
  if (it == parameters.end()) {
    throw std::runtime_error(std::string("missing parameter '") + key + "'");
  }
  return std::stod(it->second);
}

std::string hardware_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}
}  // namespace

RobStrideSystem::~RobStrideSystem()
{
  if (active_) {
    try {disable_all();} catch (...) {}
  }
  stop_executor();
}

bool RobStrideSystem::parse_bool(const std::string & value)
{
  if (value == "true" || value == "1") {return true;}
  if (value == "false" || value == "0") {return false;}
  throw std::runtime_error("boolean parameter must be true, false, 1, or 0; got '" + value + "'");
}

hardware_interface::CallbackReturn RobStrideSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    const int host = std::stoi(hardware_parameter(info, "host_can_id", "253"), nullptr, 0);
    if (host < 0 || host > 255) {throw std::runtime_error("host_can_id must be 0..255");}
    host_id_ = static_cast<uint8_t>(host);
    tx_topic_ = hardware_parameter(info, "can_tx_topic", "to_can_bus");
    rx_topic_ = hardware_parameter(info, "can_rx_topic", "from_can_bus");
    qos_depth_ = static_cast<size_t>(std::stoul(hardware_parameter(info, "can_qos_depth", "500")));
    feedback_timeout_ms_ = std::stoi(hardware_parameter(info, "feedback_timeout_ms", "3000"));
    fail_on_feedback_timeout_ = parse_bool(
      hardware_parameter(info, "fail_on_feedback_timeout", "true"));
    clear_faults_on_activate_ = parse_bool(
      hardware_parameter(info, "clear_faults_on_activate", "true"));
    set_zero_on_activate_ = parse_bool(
      hardware_parameter(info, "set_zero_on_activate", "false"));
    shutdown_stop_repetitions_ = std::stoi(
      hardware_parameter(info, "shutdown_stop_repetitions", "3"));
    shutdown_stop_interval_ms_ = std::stoi(
      hardware_parameter(info, "shutdown_stop_interval_ms", "20"));
    shutdown_confirmation_timeout_ms_ = std::stoi(
      hardware_parameter(info, "shutdown_confirmation_timeout_ms", "300"));
    startup_connection_timeout_ms_ = std::stoi(
      hardware_parameter(info, "startup_connection_timeout_ms", "3000"));
    startup_confirmation_timeout_ms_ = std::stoi(
      hardware_parameter(info, "startup_confirmation_timeout_ms", "500"));
    startup_retries_ = std::stoi(hardware_parameter(info, "startup_retries", "3"));
    if (qos_depth_ == 0 || feedback_timeout_ms_ <= 0) {
      throw std::runtime_error("can_qos_depth and feedback_timeout_ms must be positive");
    }
    if (shutdown_stop_repetitions_ <= 0 || shutdown_stop_interval_ms_ < 0 ||
      shutdown_confirmation_timeout_ms_ < 0 || startup_connection_timeout_ms_ <= 0 ||
      startup_confirmation_timeout_ms_ <= 0 || startup_retries_ <= 0)
    {
      throw std::runtime_error("invalid startup or shutdown timing parameters");
    }

    std::set<uint8_t> ids;
    joints_.reserve(info.joints.size());
    for (const auto & joint_info : info.joints) {
      Joint joint;
      if (!parse_joint(joint_info, joint)) {return hardware_interface::CallbackReturn::ERROR;}
      if (!ids.insert(joint.can_id).second) {
        throw std::runtime_error("duplicate can_id " + std::to_string(joint.can_id));
      }
      can_id_to_joint_[joint.can_id] = joints_.size();
      joints_.push_back(joint);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RobStrideSystem::parse_joint(const hardware_interface::ComponentInfo & info, Joint & joint)
{
  try {
    joint.name = info.name;
    const double unknown = std::numeric_limits<double>::quiet_NaN();
    joint.state = StateValues{unknown, unknown, unknown, unknown, 0.0};
    joint.received = joint.state;
    joint.command_position = std::numeric_limits<double>::quiet_NaN();
    const int id = std::stoi(info.parameters.at("can_id"), nullptr, 0);
    if (id < 1 || id > 255) {throw std::runtime_error("can_id must be 1..255");}
    joint.can_id = static_cast<uint8_t>(id);
    joint.limits = Limits{
      number(info.parameters, "position_min"), number(info.parameters, "position_max"),
      number(info.parameters, "velocity_min"), number(info.parameters, "velocity_max"),
      number(info.parameters, "effort_min"), number(info.parameters, "effort_max"),
      info.parameters.count("effort_wire_min") ? number(info.parameters, "effort_wire_min") : number(info.parameters, "effort_min"),
      info.parameters.count("effort_wire_max") ? number(info.parameters, "effort_wire_max") : number(info.parameters, "effort_max"),
      number(info.parameters, "kp_max"), number(info.parameters, "kd_max")};
    joint.kp = number(info.parameters, "kp");
    joint.kd = number(info.parameters, "kd");
    joint.can_timeout_ticks = static_cast<uint32_t>(
      std::stoul(info.parameters.at("can_timeout_ticks"), nullptr, 0));
    if (joint.can_timeout_ticks == 0) {
      throw std::runtime_error("can_timeout_ticks must be nonzero for fail-safe shutdown");
    }
    const auto direction = info.parameters.find("direction");
    joint.direction = direction == info.parameters.end() ? 1.0 : std::stod(direction->second);
    const auto gear = info.parameters.find("gear_ratio");
    joint.gear_ratio = gear == info.parameters.end() ? 1.0 : std::stod(gear->second);
    const auto offset = info.parameters.find("position_offset");
    joint.position_offset = offset == info.parameters.end() ? 0.0 : std::stod(offset->second);
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
      rclcpp::get_logger("RobStrideSystem"), "Joint '%s': %s", info.name.c_str(), error.what());
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
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.command_position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command_velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.command_effort);
  }
  return interfaces;
}

hardware_interface::CallbackReturn RobStrideSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    node_ = std::make_shared<rclcpp::Node>(info_.name + "_can_transport");
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_)).reliable().durability_volatile();
    publisher_ = node_->create_publisher<can_msgs::msg::Frame>(tx_topic_, qos);
    subscription_ = node_->create_subscription<can_msgs::msg::Frame>(
      rx_topic_, qos, std::bind(&RobStrideSystem::receive_frame, this, std::placeholders::_1));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spinning_ = true;
    executor_thread_ = std::thread([this]() {executor_->spin();});
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "CAN topic setup failed: %s", error.what());
    stop_executor();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_executor();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!wait_for_transport()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_) {
    if (clear_faults_on_activate_) {publish(make_stop(joint.can_id, host_id_, true));}
    if (!write_and_confirm_parameter(joint, kIndexCanTimeout, joint.can_timeout_ticks) ||
      !write_and_confirm_parameter(joint, kIndexRunMode, 0))
    {
      disable_all();
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (set_zero_on_activate_) {publish(make_set_zero(joint.can_id, host_id_));}
  }

  if (!enable_and_confirm_all()) {
    disable_all();
    return hardware_interface::CallbackReturn::ERROR;
  }
  {
    // Controllers read the exported state variables without this transport mutex.
    // Seed them before activation, then update them only from read().
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & joint : joints_) {
      joint.state = joint.received;
    }
  }
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
  stop_executor();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_error(const rclcpp_lifecycle::State &)
{
  disable_all();
  stop_executor();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobStrideSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  const auto now = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(feedback_timeout_ms_);
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & joint : joints_) {
    if (joint.feedback_received) {
      joint.state = joint.received;
    }
    if (!fail_on_feedback_timeout_) {continue;}
    const auto reference = joint.feedback_received ? joint.last_feedback : activated_at_;
    if (now - reference > timeout) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000, "Feedback timeout for joint '%s'",
        joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & joint : joints_) {
    const double joint_position = joint.position_active && std::isfinite(joint.command_position) ?
      joint.command_position : joint.state.position;
    const double motor_position = std::isfinite(joint_position) ?
      joint.direction * (joint_position - joint.position_offset) * joint.gear_ratio : 0.0;
    const double motor_velocity = joint.velocity_active && std::isfinite(joint.command_velocity) ?
      joint.direction * joint.command_velocity * joint.gear_ratio : 0.0;
    const double motor_effort = joint.effort_active && std::isfinite(joint.command_effort) ?
      joint.direction * joint.command_effort : 0.0;
    const double kp = joint.position_active ? joint.kp : 0.0;
    const double kd = (joint.position_active || joint.velocity_active) ? joint.kd : 0.0;
    publish(make_motion_command(
        joint.can_id, joint.limits, motor_position, motor_velocity, motor_effort, kp, kd));
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<CommandModeState> current_states;
  current_states.reserve(joints_.size());
  for (const auto & joint : joints_) {
    current_states.push_back(CommandModeState{
      joint.name, joint.position_active, joint.velocity_active, joint.effort_active});
  }

  std::string validation_error;
  if (!validate_command_mode_switch(current_states, start, stop, &validation_error)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & key : start) {
    for (const auto & joint : joints_) {
      if (key == joint.name + "/position" && !joint.feedback_received) {
        RCLCPP_ERROR(
          rclcpp::get_logger("RobStrideSystem"),
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
  std::lock_guard<std::mutex> lock(mutex_);
  const auto set_active = [this](const std::string & key, bool value) {
      for (auto & joint : joints_) {
        if (key == joint.name + "/position") {
          joint.position_active = value;
          if (value) {joint.command_position = joint.state.position;}
        } else if (key == joint.name + "/velocity") {
          joint.velocity_active = value;
          if (value) {joint.command_velocity = 0.0;}
        } else if (key == joint.name + "/effort") {
          joint.effort_active = value;
          if (value) {joint.command_effort = 0.0;}
        }
      }
    };
  for (const auto & key : stop) {set_active(key, false);}
  for (const auto & key : start) {set_active(key, true);}
  return hardware_interface::return_type::OK;
}

void RobStrideSystem::receive_frame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  if (msg->is_error || msg->is_rtr) {return;}
  const uint16_t area = static_cast<uint16_t>((msg->id >> 8) & 0xffff);
  const uint8_t motor_id = static_cast<uint8_t>(area & 0xff);
  const auto found = can_id_to_joint_.find(motor_id);
  if (found == can_id_to_joint_.end()) {return;}

  const auto parameter = decode_parameter_response(
    msg->id, msg->data, msg->dlc, msg->is_extended, host_id_);
  if (parameter) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & joint = joints_[found->second];
    joint.parameter_received = true;
    joint.last_parameter_index = parameter->index;
    joint.last_parameter_value = parameter->value;
    joint.last_parameter_response = std::chrono::steady_clock::now();
    feedback_condition_.notify_all();
    return;
  }

  const auto decoded = decode_feedback(
    msg->id, msg->data, msg->dlc, msg->is_extended, host_id_, joints_[found->second].limits);
  if (!decoded) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  auto & joint = joints_[found->second];
  joint.received.position =
    joint.direction * (decoded->position / joint.gear_ratio) + joint.position_offset;
  joint.received.velocity = joint.direction * (decoded->velocity / joint.gear_ratio);
  joint.received.effort = joint.direction * decoded->effort;
  joint.received.temperature = decoded->temperature;
  joint.received.fault = decoded->fault_flags;
  joint.feedback_mode = decoded->mode;
  joint.feedback_received = true;
  joint.last_feedback = std::chrono::steady_clock::now();
  feedback_condition_.notify_all();
}

void RobStrideSystem::publish(const Frame & source)
{
  if (!publisher_) {return;}
  can_msgs::msg::Frame message;
  message.header.stamp = node_->now();
  message.id = source.id;
  message.is_rtr = false;
  message.is_extended = true;
  message.is_error = false;
  message.dlc = 8;
  message.data = source.data;
  publisher_->publish(message);
}

bool RobStrideSystem::wait_for_transport()
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(startup_connection_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (publisher_ && subscription_ && publisher_->get_subscription_count() > 0 &&
      subscription_->get_publisher_count() > 0)
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(
    node_->get_logger(),
    "Timed out waiting for ros2_socketcan publishers/subscribers on '%s' and '%s'",
    tx_topic_.c_str(), rx_topic_.c_str());
  return false;
}

bool RobStrideSystem::write_and_confirm_parameter(
  Joint & joint, uint16_t index, uint32_t value)
{
  for (int attempt = 1; attempt <= startup_retries_; ++attempt) {
    const auto requested_at = std::chrono::steady_clock::now();
    if (index == kIndexRunMode) {
      publish(make_write_u8(joint.can_id, host_id_, index, static_cast<uint8_t>(value)));
    } else {
      publish(make_write_u32(joint.can_id, host_id_, index, value));
    }
    publish(make_read_parameter(joint.can_id, host_id_, index));

    std::unique_lock<std::mutex> lock(mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, std::chrono::milliseconds(startup_confirmation_timeout_ms_),
      [&joint, index, value, requested_at]() {
        return joint.parameter_received && joint.last_parameter_response >= requested_at &&
               joint.last_parameter_index == index && joint.last_parameter_value == value;
      });
    if (confirmed) {return true;}
    RCLCPP_WARN(
      node_->get_logger(),
      "Joint '%s': parameter 0x%04X confirmation attempt %d/%d failed",
      joint.name.c_str(), index, attempt, startup_retries_);
  }
  RCLCPP_ERROR(
    node_->get_logger(), "Joint '%s': parameter 0x%04X could not be confirmed",
    joint.name.c_str(), index);
  return false;
}

bool RobStrideSystem::enable_and_confirm_all()
{
  for (int attempt = 1; attempt <= startup_retries_; ++attempt) {
    const auto requested_at = std::chrono::steady_clock::now();
    for (const auto & joint : joints_) {
      publish(make_enable(joint.can_id, host_id_));
      publish(make_motion_command(
          joint.can_id, joint.limits, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    std::unique_lock<std::mutex> lock(mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, std::chrono::milliseconds(startup_confirmation_timeout_ms_),
      [this, requested_at]() {
        return std::all_of(joints_.begin(), joints_.end(), [requested_at](const Joint & joint) {
          return joint.feedback_received && joint.last_feedback >= requested_at &&
                 joint.feedback_mode == 2;
        });
      });
    if (confirmed) {return true;}
    RCLCPP_WARN(
      node_->get_logger(), "Motor enable confirmation attempt %d/%d failed",
      attempt, startup_retries_);
  }
  RCLCPP_ERROR(node_->get_logger(), "Not all motors entered Run mode");
  return false;
}

void RobStrideSystem::disable_all()
{
  if (!publisher_) {
    active_ = false;
    return;
  }
  active_ = false;
  const auto stop_started = std::chrono::steady_clock::now();
  for (int attempt = 0; attempt < shutdown_stop_repetitions_; ++attempt) {
    for (const auto & joint : joints_) {
      publish(make_motion_command(joint.can_id, joint.limits, 0.0, 0.0, 0.0, 0.0, 0.0));
      publish(make_stop(joint.can_id, host_id_));
    }
    if (publisher_->get_subscription_count() > 0) {
      (void)publisher_->wait_for_all_acked(std::chrono::milliseconds(50));
    }
    if (attempt + 1 < shutdown_stop_repetitions_ && shutdown_stop_interval_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(shutdown_stop_interval_ms_));
    }
  }

  if (shutdown_confirmation_timeout_ms_ > 0) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, std::chrono::milliseconds(shutdown_confirmation_timeout_ms_),
      [this, stop_started]() {
        return std::all_of(joints_.begin(), joints_.end(), [stop_started](const Joint & joint) {
          return joint.feedback_received && joint.last_feedback >= stop_started &&
                 joint.feedback_mode == 0;
        });
      });
    if (!confirmed) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Motor stop was not confirmed on CAN; motor watchdog must force reset after timeout");
    }
  }
}

void RobStrideSystem::stop_executor()
{
  active_ = false;
  if (executor_) {executor_->cancel();}
  if (executor_thread_.joinable()) {executor_thread_.join();}
  spinning_ = false;
  subscription_.reset();
  publisher_.reset();
  if (executor_ && node_) {executor_->remove_node(node_);}
  executor_.reset();
  node_.reset();
}

}  // namespace robstride_ros2

PLUGINLIB_EXPORT_CLASS(robstride_ros2::RobStrideSystem, hardware_interface::SystemInterface)
