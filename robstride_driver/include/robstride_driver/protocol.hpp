#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace robstride_driver
{

struct Limits
{
  double position_min;
  double position_max;
  double velocity_min;
  double velocity_max;
  double effort_min;
  double effort_max;
  double effort_wire_min;
  double effort_wire_max;
  double kp_max;
  double kd_max;
};

struct Frame
{
  uint32_t id{0};
  std::array<uint8_t, 8> data{};
};

struct Feedback
{
  uint8_t motor_id{0};
  uint8_t mode{0};
  uint8_t fault_flags{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double temperature{0.0};
};

struct ParameterResponse
{
  uint8_t motor_id{0};
  uint16_t index{0};
  uint32_t value{0};
};

constexpr uint8_t kTypeMotionControl = 1;
constexpr uint8_t kTypeFeedback = 2;
constexpr uint8_t kTypeEnable = 3;
constexpr uint8_t kTypeStop = 4;
constexpr uint8_t kTypeSetZero = 6;
constexpr uint8_t kTypeReadParameter = 17;
constexpr uint8_t kTypeWriteParameter = 18;
constexpr uint8_t kMotorModeReset = 0;
constexpr uint8_t kMotorModeRun = 2;
constexpr uint16_t kIndexRunMode = 0x7005;
constexpr uint16_t kIndexCanTimeout = 0x7028;

uint16_t encode_u16(double value, double minimum, double maximum);
double decode_u16(uint16_t value, double minimum, double maximum);
Frame make_motion_command(
  uint8_t motor_id, const Limits & limits, double position, double velocity, double effort,
  double kp, double kd);
Frame make_enable(uint8_t motor_id, uint8_t host_id);
Frame make_stop(uint8_t motor_id, uint8_t host_id, bool clear_fault = false);
Frame make_set_zero(uint8_t motor_id, uint8_t host_id);
Frame make_read_parameter(uint8_t motor_id, uint8_t host_id, uint16_t index);
Frame make_write_u8(uint8_t motor_id, uint8_t host_id, uint16_t index, uint8_t value);
Frame make_write_u32(uint8_t motor_id, uint8_t host_id, uint16_t index, uint32_t value);
Frame make_write_float(uint8_t motor_id, uint8_t host_id, uint16_t index, float value);
std::optional<Feedback> decode_feedback(
  uint32_t id, const std::array<uint8_t, 8> & data, uint8_t dlc, bool is_extended,
  uint8_t expected_host_id, const Limits & limits);
std::optional<ParameterResponse> decode_parameter_response(
  uint32_t id, const std::array<uint8_t, 8> & data, uint8_t dlc, bool is_extended,
  uint8_t expected_host_id);

}  // namespace robstride_driver
