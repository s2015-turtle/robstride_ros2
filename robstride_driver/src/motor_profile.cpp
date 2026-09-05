#include "robstride_driver/motor_profile.hpp"

#include <stdexcept>
#include <string>

namespace robstride_driver
{
namespace
{
// English manuals dated 2026-07-13, RobStride/Product_Information:
// https://github.com/RobStride/Product_Information/tree/main/Product%20Literature
// Command/feedback normalization ranges, not recommended operating limits.
struct Profile
{
  std::string_view name;
  double velocity;
  double effort;
  double kp;
  double kd;
};
constexpr Profile profiles[] = {
  {"RS00", 33.0, 14.0, 500.0, 5.0},
  {"RS01", 44.0, 17.0, 500.0, 5.0},
  {"RS02", 44.0, 17.0, 500.0, 5.0},
  {"RS03", 20.0, 60.0, 5000.0, 100.0},
  {"RS04", 15.0, 120.0, 5000.0, 100.0},
  {"RS05", 50.0, 5.5, 500.0, 5.0},
  {"RS06", 50.0, 36.0, 5000.0, 100.0},
  {"EL05", 50.0, 6.0, 500.0, 5.0},
};
}  // namespace

Limits motor_profile(std::string_view model)
{
  if (model == "EduLite05") {model = "EL05";}
  for (const auto & profile : profiles) {
    if (profile.name == model) {
      return Limits{
        -12.566370614, 12.566370614, -profile.velocity, profile.velocity,
        -profile.effort, profile.effort, -profile.effort, profile.effort,
        profile.kp, profile.kd};
    }
  }
  throw std::invalid_argument("unknown motor model '" + std::string(model) + "'");
}
}  // namespace robstride_driver
