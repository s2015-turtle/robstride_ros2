#pragma once

#include <string>
#include <vector>

namespace robstride_ros2_control
{

struct CommandModeState
{
  std::string joint_name;
  bool position_active{false};
  bool velocity_active{false};
  bool effort_active{false};
};

// Evaluates the state that would result after applying stop_interfaces followed by
// start_interfaces. Ignores foreign joints; returns false for unsupported
// interfaces on owned joints or multiple active command interfaces on one joint.
bool validate_command_mode_switch(
  const std::vector<CommandModeState> & current_states,
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces,
  std::string * error_message = nullptr);

std::vector<CommandModeState> command_modes_after_switch(
  const std::vector<CommandModeState> & current_states,
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces);

}  // namespace robstride_ros2_control
