#include "robstride_ros2_control/robstride_system.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "robstride_ros2_control/command_mode.hpp"
#include "robstride_ros2_control/driver_config.hpp"
#include "robstride_driver/driver.hpp"
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
#include "robstride_ros2_control/interface_value_bindings.hpp"
#endif

namespace robstride_ros2_control
{
namespace
{
std::vector<CommandModeState> current_command_modes(robstride_driver::RobStrideDriver & driver)
{
  const auto modes = driver.command_modes();
  const auto & joints = driver.joints();
  std::vector<CommandModeState> states;
  states.reserve(joints.size());
  for (size_t index = 0; index < joints.size(); ++index) {
    states.push_back(CommandModeState{
      joints[index].name, modes[index].position, modes[index].velocity, modes[index].effort});
  }
  return states;
}
}  // namespace

struct RobStrideSystem::Impl
{
  Impl()
  : driver(rclcpp::get_logger("RobStrideSystem")) {}

  robstride_driver::RobStrideDriver driver;
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
  InterfaceValueBindings<hardware_interface::StateInterface> states;
  InterfaceValueBindings<hardware_interface::CommandInterface> commands;
  bool command_sync_failed{false};
#endif
};

RobStrideSystem::RobStrideSystem()
: impl_(std::make_unique<Impl>())
{
}

RobStrideSystem::~RobStrideSystem() = default;

#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
hardware_interface::CallbackReturn RobStrideSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    return impl_->driver.initialize(
      parse_driver_configuration(params.hardware_info)) ?
           hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
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
  try {
    return impl_->driver.initialize(
      parse_driver_configuration(info)) ?
           hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}
#endif

#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
std::vector<hardware_interface::StateInterface::ConstSharedPtr>
RobStrideSystem::on_export_state_interfaces()
{
  auto interfaces = hardware_interface::SystemInterface::on_export_state_interfaces();
  impl_->states.clear();
  for (auto & joint : impl_->driver.joints()) {
    impl_->states.add(get_state_interface_handle(joint.name + "/position"), joint.state.position);
    impl_->states.add(get_state_interface_handle(joint.name + "/velocity"), joint.state.velocity);
    impl_->states.add(get_state_interface_handle(joint.name + "/effort"), joint.state.effort);
    if (joint.exports_temperature) {
      impl_->states.add(
        get_state_interface_handle(joint.name + "/temperature"), joint.state.temperature);
    }
    if (joint.exports_fault) {
      impl_->states.add(get_state_interface_handle(joint.name + "/fault"), joint.state.fault);
    }
  }
  if (!impl_->states.publish()) {throw std::runtime_error("Failed to initialize state interfaces");}
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
RobStrideSystem::on_export_command_interfaces()
{
  auto interfaces = hardware_interface::SystemInterface::on_export_command_interfaces();
  impl_->commands.clear();
  for (auto & joint : impl_->driver.joints()) {
    impl_->commands.add(
      get_command_interface_handle(joint.name + "/position"), joint.command.position);
    impl_->commands.add(
      get_command_interface_handle(joint.name + "/velocity"), joint.command.velocity);
    impl_->commands.add(get_command_interface_handle(joint.name + "/effort"), joint.command.effort);
  }
  if (!impl_->commands.publish()) {
    throw std::runtime_error("Failed to initialize command interfaces");
  }
  return interfaces;
}
#else
std::vector<hardware_interface::StateInterface> RobStrideSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : impl_->driver.joints()) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.state.position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.state.effort);
    if (joint.exports_temperature) {
      interfaces.emplace_back(joint.name, "temperature", &joint.state.temperature);
    }
    if (joint.exports_fault) {
      interfaces.emplace_back(joint.name, "fault", &joint.state.fault);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobStrideSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : impl_->driver.joints()) {
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.command.position);
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity);
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_EFFORT, &joint.command.effort);
  }
  return interfaces;
}

#endif

hardware_interface::CallbackReturn RobStrideSystem::on_configure(const rclcpp_lifecycle::State &)
{
  return impl_->driver.open() ?
         hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn RobStrideSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  impl_->driver.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!impl_->driver.start()) {return hardware_interface::CallbackReturn::ERROR;}
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
  if (!impl_->states.publish() || !impl_->commands.publish()) {
    impl_->driver.stop();
    return hardware_interface::CallbackReturn::ERROR;
  }
  impl_->command_sync_failed = false;
#endif
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  impl_->driver.stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  impl_->driver.stop();
  impl_->driver.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobStrideSystem::on_error(const rclcpp_lifecycle::State &)
{
  impl_->driver.stop();
  impl_->driver.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobStrideSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
  if (impl_->command_sync_failed) {return hardware_interface::return_type::ERROR;}
#endif
  if (!impl_->driver.update_state()) {return hardware_interface::return_type::ERROR;}
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
  if (!impl_->states.publish()) {return hardware_interface::return_type::ERROR;}
#endif
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
  if (impl_->command_sync_failed) {return hardware_interface::return_type::ERROR;}
  if (!impl_->commands.consume()) {return hardware_interface::return_type::ERROR;}
#endif
  return impl_->driver.send_commands() ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type RobStrideSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  std::string validation_error;
  if (!validate_command_mode_switch(
      current_command_modes(impl_->driver), start, stop, &validation_error))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  const auto resulting_states = command_modes_after_switch(
    current_command_modes(impl_->driver), start, stop);
  std::vector<robstride_driver::ClaimedInterfaces> modes;
  modes.reserve(resulting_states.size());
  for (const auto & state : resulting_states) {
    modes.push_back(robstride_driver::ClaimedInterfaces{
      state.position_active, state.velocity_active, state.effort_active});
  }
  if (!impl_->driver.validate_command_modes(modes, &validation_error)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
  if (impl_->command_sync_failed) {return hardware_interface::return_type::ERROR;}
#endif
  const auto previous_states = current_command_modes(impl_->driver);
  const auto resulting_states = command_modes_after_switch(previous_states, start, stop);
  std::vector<robstride_driver::ClaimedInterfaces> modes;
  modes.reserve(resulting_states.size());
  for (const auto & state : resulting_states) {
    modes.push_back(robstride_driver::ClaimedInterfaces{
      state.position_active, state.velocity_active, state.effort_active});
  }
  std::string validation_error;
  if (!impl_->driver.apply_command_modes(modes, &validation_error)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"), "Command mode switch rejected: %s",
      validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }
#ifdef ROBSTRIDE_ROS2_USE_MANAGED_INTERFACES
  // apply_command_modes initializes newly claimed targets in driver storage.
  // Publish those targets before write() can consume older framework values.
  std::vector<std::string> initialized_commands;
  for (size_t index = 0; index < resulting_states.size(); ++index) {
    const auto & previous = previous_states[index];
    const auto & current = resulting_states[index];
    if (!previous.position_active && current.position_active) {
      initialized_commands.push_back(current.joint_name + "/position");
    }
    if (!previous.velocity_active && current.velocity_active) {
      initialized_commands.push_back(current.joint_name + "/velocity");
    }
    if (!previous.effort_active && current.effort_active) {
      initialized_commands.push_back(current.joint_name + "/effort");
    }
  }
  if (!impl_->commands.publish_selected(initialized_commands)) {
    // perform_command_mode_switch() errors do not necessarily invoke on_error().
    // Prevent the next write from replacing initialized targets with stale values;
    // the next read/write ERROR delegates stopping to the hardware lifecycle.
    impl_->command_sync_failed = true;
    RCLCPP_ERROR(
      rclcpp::get_logger("RobStrideSystem"),
      "Failed to synchronize initial command values; hardware reactivation is required");
    return hardware_interface::return_type::ERROR;
  }
#endif
  return hardware_interface::return_type::OK;
}

}  // namespace robstride_ros2_control

PLUGINLIB_EXPORT_CLASS(robstride_ros2_control::RobStrideSystem, hardware_interface::SystemInterface)
