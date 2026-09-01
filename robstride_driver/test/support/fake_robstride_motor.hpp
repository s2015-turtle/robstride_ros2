#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

namespace robstride_driver::test
{

struct FakeMotorOptions
{
  const char * interface_name{"vcan0"};
  uint8_t motor_id{1};
  uint8_t host_id{0xfd};
};

class FakeRobStrideMotor
{
public:
  explicit FakeRobStrideMotor(FakeMotorOptions options);
  ~FakeRobStrideMotor();

  FakeRobStrideMotor(const FakeRobStrideMotor &) = delete;
  FakeRobStrideMotor & operator=(const FakeRobStrideMotor &) = delete;

  void set_feedback_enabled(bool enabled);
  void set_parameter_confirmation_enabled(bool enabled);
  void set_stop_confirmation_enabled(bool enabled);
  void set_response_delay(std::chrono::milliseconds delay);
  void report_reset();

  uint8_t mode() const;
  uint64_t enable_count() const;
  uint64_t motion_count() const;
  uint64_t stop_count() const;
  uint32_t parameter(uint16_t index) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robstride_driver::test
