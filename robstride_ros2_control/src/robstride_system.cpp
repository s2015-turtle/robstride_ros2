#include "robstride_ros2_control/robstride_system.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "robstride_ros2_control/command_mode.hpp"
#include "robstride_ros2_control/driver_config.hpp"
#include "robstride_driver/driver.hpp"

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
  return impl_->driver.start() ?
         hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
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
  return impl_->driver.update_state() ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type RobStrideSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  impl_->driver.send_commands();
  return hardware_interface::return_type::OK;
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

  const auto feedback = impl_->driver.feedback_received();
  const auto & joints = impl_->driver.joints();
  for (size_t index = 0; index < joints.size(); ++index) {
    if (!feedback[index] &&
      std::find(start.begin(), start.end(), joints[index].name + "/position") != start.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobStrideSystem"),
        "Cannot start position command interface for '%s' before first feedback",
        joints[index].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobStrideSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  const auto resulting_states = command_modes_after_switch(
    current_command_modes(impl_->driver), start, stop);
  std::vector<robstride_driver::ClaimedInterfaces> modes;
  modes.reserve(resulting_states.size());
  for (const auto & state : resulting_states) {
    modes.push_back(robstride_driver::ClaimedInterfaces{
      state.position_active, state.velocity_active, state.effort_active});
  }
  return impl_->driver.apply_command_modes(modes) ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

}  // namespace robstride_ros2_control

PLUGINLIB_EXPORT_CLASS(robstride_ros2_control::RobStrideSystem, hardware_interface::SystemInterface)
