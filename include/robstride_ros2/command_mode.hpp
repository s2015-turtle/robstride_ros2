#pragma once

#include <string>
#include <vector>

namespace robstride_ros2
{

struct CommandModeState
{
  std::string joint_name;
  bool position_active{false};
  bool velocity_active{false};
  bool effort_active{false};
};

// Evaluates the state that would result after applying stop_interfaces followed by
// start_interfaces. Returns false for unknown interface keys or if more than one
// command interface would be active for any one joint.
bool validate_command_mode_switch(
  const std::vector<CommandModeState> & current_states,
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces,
  std::string * error_message = nullptr);

}  // namespace robstride_ros2
