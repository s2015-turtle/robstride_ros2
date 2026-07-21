#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#endif
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace robstride_ros2_control
{

// ros2_control adapter for the internal RobStride driver. Protocol handling,
// transport scheduling, recovery, and motor lifecycle operations stay outside
// this plugin-facing class.
class RobStrideSystem : public hardware_interface::SystemInterface
{
public:
  RobStrideSystem();
  ~RobStrideSystem() override;

#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#endif
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robstride_ros2_control
