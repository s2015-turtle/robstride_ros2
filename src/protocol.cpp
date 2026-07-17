#include "robstride_ros2/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace robstride_ros2
{
namespace
{
uint32_t base_id(uint8_t type, uint16_t data_area_2, uint8_t destination)
{
  return (static_cast<uint32_t>(type & 0x1f) << 24) |
         (static_cast<uint32_t>(data_area_2) << 8) | destination;
}

void put_be16(std::array<uint8_t, 8> & data, size_t offset, uint16_t value)
{
  data[offset] = static_cast<uint8_t>(value >> 8);
  data[offset + 1] = static_cast<uint8_t>(value & 0xff);
}
}  // namespace

uint16_t encode_u16(double value, double minimum, double maximum)
{
  if (!std::isfinite(value) || !(minimum < maximum)) {
    return 0;
  }
  const double normalized = (std::clamp(value, minimum, maximum) - minimum) / (maximum - minimum);
  return static_cast<uint16_t>(std::lround(normalized * 65535.0));
}

double decode_u16(uint16_t value, double minimum, double maximum)
{
  return minimum + (static_cast<double>(value) / 65535.0) * (maximum - minimum);
}

Frame make_motion_command(
  uint8_t motor_id, const Limits & limits, double position, double velocity, double effort,
  double kp, double kd)
{
  Frame frame;
  const double safe_effort = std::clamp(effort, limits.effort_min, limits.effort_max);
  const uint16_t effort_raw = encode_u16(safe_effort, limits.effort_wire_min, limits.effort_wire_max);
  frame.id = base_id(kTypeMotionControl, effort_raw, motor_id);
  put_be16(frame.data, 0, encode_u16(position, limits.position_min, limits.position_max));
  put_be16(frame.data, 2, encode_u16(velocity, limits.velocity_min, limits.velocity_max));
  put_be16(frame.data, 4, encode_u16(kp, 0.0, limits.kp_max));
  put_be16(frame.data, 6, encode_u16(kd, 0.0, limits.kd_max));
  return frame;
}

Frame make_enable(uint8_t motor_id, uint8_t host_id)
{
  Frame frame;
  frame.id = base_id(kTypeEnable, host_id, motor_id);
  return frame;
}

Frame make_stop(uint8_t motor_id, uint8_t host_id, bool clear_fault)
{
  Frame frame;
  frame.id = base_id(kTypeStop, host_id, motor_id);
  frame.data[0] = clear_fault ? 1 : 0;
  return frame;
}

Frame make_set_zero(uint8_t motor_id, uint8_t host_id)
{
  Frame frame;
  frame.id = base_id(kTypeSetZero, host_id, motor_id);
  frame.data[0] = 1;
  return frame;
}

Frame make_write_u8(uint8_t motor_id, uint8_t host_id, uint16_t index, uint8_t value)
{
  Frame frame;
  frame.id = base_id(kTypeWriteParameter, host_id, motor_id);
  frame.data[0] = static_cast<uint8_t>(index & 0xff);
  frame.data[1] = static_cast<uint8_t>(index >> 8);
  frame.data[4] = value;
  return frame;
}

Frame make_write_u32(uint8_t motor_id, uint8_t host_id, uint16_t index, uint32_t value)
{
  Frame frame = make_write_u8(motor_id, host_id, index, 0);
  for (size_t i = 0; i < 4; ++i) {
    frame.data[4 + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xff);
  }
  return frame;
}

Frame make_write_float(uint8_t motor_id, uint8_t host_id, uint16_t index, float value)
{
  Frame frame = make_write_u8(motor_id, host_id, index, 0);
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  for (size_t i = 0; i < 4; ++i) {
    frame.data[4 + i] = static_cast<uint8_t>((raw >> (8 * i)) & 0xff);
  }
  return frame;
}

std::optional<Feedback> decode_feedback(
  uint32_t id, const std::array<uint8_t, 8> & data, uint8_t dlc, bool is_extended,
  uint8_t expected_host_id, const Limits & limits)
{
  if (!is_extended || dlc != 8 || ((id >> 24) & 0x1f) != kTypeFeedback ||
    (id & 0xff) != expected_host_id)
  {
    return std::nullopt;
  }
  const uint16_t area = static_cast<uint16_t>((id >> 8) & 0xffff);
  const auto get_be16 = [&data](size_t offset) {
      return static_cast<uint16_t>((static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1]);
    };
  Feedback feedback;
  feedback.motor_id = static_cast<uint8_t>(area & 0xff);
  feedback.mode = static_cast<uint8_t>((area >> 14) & 0x03);
  feedback.fault_flags = static_cast<uint8_t>((area >> 8) & 0x3f);
  feedback.position = decode_u16(get_be16(0), limits.position_min, limits.position_max);
  feedback.velocity = decode_u16(get_be16(2), limits.velocity_min, limits.velocity_max);
  feedback.effort = decode_u16(get_be16(4), limits.effort_wire_min, limits.effort_wire_max);
  feedback.temperature = static_cast<double>(get_be16(6)) * 0.1;
  return feedback;
}

}  // namespace robstride_ros2
