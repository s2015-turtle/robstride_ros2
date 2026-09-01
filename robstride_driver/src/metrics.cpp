#include "robstride_driver/metrics.hpp"

namespace robstride_driver
{
namespace
{
double rate(uint64_t count, std::chrono::nanoseconds period) noexcept
{
  const double seconds = std::chrono::duration<double>(period).count();
  return seconds > 0.0 ? static_cast<double>(count) / seconds : 0.0;
}
}  // namespace

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
