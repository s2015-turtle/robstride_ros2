#include "robstride_ros2/command_mode.hpp"

#include <algorithm>

namespace robstride_ros2
{
namespace
{
bool set_interface(
  std::vector<CommandModeState> & states, const std::string & key, bool active)
{
  for (auto & state : states) {
    if (key == state.joint_name + "/position") {
      state.position_active = active;
      return true;
    }
    if (key == state.joint_name + "/velocity") {
      state.velocity_active = active;
      return true;
    }
    if (key == state.joint_name + "/effort") {
      state.effort_active = active;
      return true;
    }
  }
  return false;
}
}  // namespace

bool validate_command_mode_switch(
  const std::vector<CommandModeState> & current_states,
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces,
  std::string * error_message)
{
  auto resulting_states = current_states;
  for (const auto & key : stop_interfaces) {
    if (!set_interface(resulting_states, key, false)) {
      if (error_message) {*error_message = "unknown command interface '" + key + "'";}
      return false;
    }
  }
  for (const auto & key : start_interfaces) {
    if (!set_interface(resulting_states, key, true)) {
      if (error_message) {*error_message = "unknown command interface '" + key + "'";}
      return false;
    }
  }

  for (const auto & state : resulting_states) {
    const int active_count = static_cast<int>(state.position_active) +
      static_cast<int>(state.velocity_active) + static_cast<int>(state.effort_active);
    if (active_count > 1) {
      if (error_message) {
        *error_message = "joint '" + state.joint_name +
          "' would have multiple active command interfaces";
      }
      return false;
    }
  }
  return true;
}

}  // namespace robstride_ros2
