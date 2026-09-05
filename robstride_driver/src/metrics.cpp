#include "robstride_driver/metrics.hpp"

#include <cstring>
#include <sstream>

namespace robstride_driver
{
namespace
{
double rate(uint64_t count, std::chrono::nanoseconds period) noexcept
{
  const double seconds = std::chrono::duration<double>(period).count();
  return seconds > 0.0 ? static_cast<double>(count) / seconds : 0.0;
}

uint64_t double_bits(double value) noexcept
{
  uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double bits_double(uint64_t bits) noexcept
{
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

AtomicMotorFeedback::AtomicMotorFeedback() noexcept
{
  store(MotorFeedbackSample{});
}

void AtomicMotorFeedback::store(const MotorFeedbackSample & sample) noexcept
{
  sequence_.fetch_add(1, std::memory_order_seq_cst);
  count_.store(sample.count, std::memory_order_relaxed);
  last_received_at_ns_.store(sample.last_received_at_ns, std::memory_order_relaxed);
  maximum_gap_ns_.store(sample.maximum_gap_ns, std::memory_order_relaxed);
  mode_.store(sample.mode, std::memory_order_relaxed);
  fault_flags_.store(sample.fault_flags, std::memory_order_relaxed);
  temperature_bits_.store(double_bits(sample.temperature), std::memory_order_relaxed);
  sequence_.fetch_add(1, std::memory_order_seq_cst);
}

MotorFeedbackSample AtomicMotorFeedback::load() const noexcept
{
  MotorFeedbackSample sample;
  while (true) {
    const uint64_t before = sequence_.load(std::memory_order_seq_cst);
    if ((before & 1u) != 0u) {continue;}
    sample.count = count_.load(std::memory_order_relaxed);
    sample.last_received_at_ns = last_received_at_ns_.load(std::memory_order_relaxed);
    sample.maximum_gap_ns = maximum_gap_ns_.load(std::memory_order_relaxed);
    sample.mode = mode_.load(std::memory_order_relaxed);
    sample.fault_flags = fault_flags_.load(std::memory_order_relaxed);
    sample.temperature = bits_double(temperature_bits_.load(std::memory_order_relaxed));
    const uint64_t after = sequence_.load(std::memory_order_seq_cst);
    if (before == after) {return sample;}
  }
}

const char * motor_mode_name(uint8_t mode) noexcept
{
  switch (mode) {
    case 0: return "Reset";
    case 1: return "Calibration";
    case 2: return "Run";
    default: return "Unknown";
  }
}

const char * transport_health_name(CanTransportHealthState state) noexcept
{
  switch (state) {
    case CanTransportHealthState::healthy: return "healthy";
    case CanTransportHealthState::bridge_unavailable: return "bridge unavailable";
    case CanTransportHealthState::transmit_stalled: return "transmit stalled";
    case CanTransportHealthState::worker_stopped: return "worker stopped";
    default: return "unknown";
  }
}

std::vector<std::string> motor_fault_names(uint8_t fault_flags)
{
  // Normal private-protocol feedback carries bits 21..16 in a shifted six-bit mask.
  // From low to high, the manuals define undervoltage, phase overcurrent,
  // over-temperature, magnetic encoder, stall overload, and uncalibrated encoder.
  static constexpr const char * kNames[] = {
    "undervoltage", "overcurrent", "over-temperature", "encoder fault",
    "stall overload", "encoder uncalibrated"};
  std::vector<std::string> names;
  for (uint8_t bit = 0; bit < 6; ++bit) {
    if ((fault_flags & (1u << bit)) != 0u) {names.emplace_back(kNames[bit]);}
  }
  return names;
}

std::string motor_fault_summary(uint8_t fault_flags)
{
  const auto names = motor_fault_names(fault_flags);
  if (names.empty()) {return "none";}
  std::ostringstream summary;
  for (size_t index = 0; index < names.size(); ++index) {
    if (index != 0) {summary << ", ";}
    summary << names[index];
  }
  return summary.str();
}

double CanTransportMetrics::transmit_rate_hz() const noexcept
{
  return rate(transmitted_frames(), observation_period);
}

double CanTransportMetrics::motion_rate_hz() const noexcept
{
  return rate(motion_frames_transmitted, observation_period);
}

double DriverMetrics::receive_rate_hz() const noexcept
{
  return rate(received_frames(), observation_period);
}

}  // namespace robstride_driver
