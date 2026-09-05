#pragma once

#include <string_view>

#include "robstride_driver/protocol.hpp"

namespace robstride_driver
{
// Throws std::invalid_argument for an unknown model. Custom limits are supplied
// by the caller, not inferred from another motor model.
Limits motor_profile(std::string_view model);
}  // namespace robstride_driver
