#include "fake_robstride_motor.hpp"

#include <linux/can.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "robstride_driver/protocol.hpp"
#include "vcan_socket.hpp"

using namespace std::chrono_literals;

namespace robstride_driver::test
{
namespace
{
uint16_t read_be16(const std::array<uint8_t, 8> & data, size_t offset)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1]);
}

void put_be16(std::array<uint8_t, 8> & data, size_t offset, uint16_t value)
{
  data[offset] = static_cast<uint8_t>(value >> 8);
  data[offset + 1] = static_cast<uint8_t>(value & 0xff);
}
}  // namespace

struct FakeRobStrideMotor::Impl
{
  explicit Impl(FakeMotorOptions motor_options)
  : options(std::move(motor_options)), socket(options.interface_name), worker([this]() {run();})
  {
  }

  ~Impl()
  {
    running = false;
    if (worker.joinable()) {worker.join();}
  }

  void run()
  {
    try {
      while (running) {
        can_frame raw{};
        if (!socket.receive(raw, 20ms)) {continue;}
        if ((raw.can_id & CAN_EFF_FLAG) == 0 || raw.can_dlc != 8) {continue;}
        handle(raw.can_id & CAN_EFF_MASK, raw.data);
      }
    } catch (const std::exception & error) {
      worker_error = error.what();
      running = false;
    }
  }

  void handle(uint32_t id, const uint8_t * raw_data)
  {
    const uint8_t type = static_cast<uint8_t>((id >> 24) & 0x1f);
    const uint8_t destination = static_cast<uint8_t>(id & 0xff);
    if (destination != options.motor_id) {return;}

    std::array<uint8_t, 8> data{};
    std::copy(raw_data, raw_data + data.size(), data.begin());
    if (type == kTypeWriteParameter) {
      const uint16_t index = static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
      const uint32_t value = static_cast<uint32_t>(data[4]) |
        (static_cast<uint32_t>(data[5]) << 8) |
        (static_cast<uint32_t>(data[6]) << 16) |
        (static_cast<uint32_t>(data[7]) << 24);
      std::lock_guard<std::mutex> lock(state_mutex);
      parameters[index] = value;
    } else if (type == kTypeReadParameter) {
      if (!parameter_confirmation_enabled) {return;}
      const uint16_t index = static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
      uint32_t value = 0;
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        const auto entry = parameters.find(index);
        if (entry != parameters.end()) {value = entry->second;}
      }
      send_parameter(index, value);
    } else if (type == kTypeEnable) {
      ++enable_count;
      mode = kMotorModeRun;
      send_feedback();
    } else if (type == kTypeStop) {
      ++stop_count;
      mode = kMotorModeReset;
      if (stop_confirmation_enabled) {send_feedback();}
    } else if (type == kTypeMotionControl) {
      ++motion_count;
      position_raw = read_be16(data, 0);
      velocity_raw = read_be16(data, 2);
      effort_raw = static_cast<uint16_t>((id >> 8) & 0xffff);
      send_feedback();
    }
  }

  void send_parameter(uint16_t index, uint32_t value)
  {
    delay_response();
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(index & 0xff);
    data[1] = static_cast<uint8_t>(index >> 8);
    for (size_t offset = 0; offset < 4; ++offset) {
      data[4 + offset] = static_cast<uint8_t>((value >> (8 * offset)) & 0xff);
    }
    const uint32_t id =
      (static_cast<uint32_t>(kTypeReadParameter) << 24) |
      (static_cast<uint32_t>(options.motor_id) << 8) | options.host_id;
    socket.send(id, data);
  }

  void send_feedback()
  {
    if (!feedback_enabled) {return;}
    delay_response();
    std::array<uint8_t, 8> data{};
    put_be16(data, 0, position_raw.load());
    put_be16(data, 2, velocity_raw.load());
    put_be16(data, 4, effort_raw.load());
    put_be16(data, 6, 300);
    const uint16_t area =
      (static_cast<uint16_t>(mode.load() & 0x03) << 14) | options.motor_id;
    const uint32_t id =
      (static_cast<uint32_t>(kTypeFeedback) << 24) |
      (static_cast<uint32_t>(area) << 8) | options.host_id;
    socket.send(id, data);
  }

  void delay_response() const
  {
    const auto delay = response_delay_ms.load();
    if (delay > 0) {std::this_thread::sleep_for(std::chrono::milliseconds(delay));}
  }

  FakeMotorOptions options;
  VcanSocket socket;
  std::atomic<bool> running{true};
  std::atomic<bool> feedback_enabled{true};
  std::atomic<bool> parameter_confirmation_enabled{true};
  std::atomic<bool> stop_confirmation_enabled{true};
  std::atomic<int64_t> response_delay_ms{0};
  std::atomic<uint8_t> mode{kMotorModeReset};
  std::atomic<uint16_t> position_raw{32768};
  std::atomic<uint16_t> velocity_raw{32768};
  std::atomic<uint16_t> effort_raw{32768};
  std::atomic<uint64_t> enable_count{0};
  std::atomic<uint64_t> motion_count{0};
  std::atomic<uint64_t> stop_count{0};
  mutable std::mutex state_mutex;
  std::unordered_map<uint16_t, uint32_t> parameters;
  std::string worker_error;
  std::thread worker;
};

FakeRobStrideMotor::FakeRobStrideMotor(FakeMotorOptions options)
: impl_(std::make_unique<Impl>(std::move(options)))
{
}

FakeRobStrideMotor::~FakeRobStrideMotor() = default;

void FakeRobStrideMotor::set_feedback_enabled(bool enabled)
{
  impl_->feedback_enabled = enabled;
}

void FakeRobStrideMotor::set_parameter_confirmation_enabled(bool enabled)
{
  impl_->parameter_confirmation_enabled = enabled;
}

void FakeRobStrideMotor::set_stop_confirmation_enabled(bool enabled)
{
  impl_->stop_confirmation_enabled = enabled;
}

void FakeRobStrideMotor::set_response_delay(std::chrono::milliseconds delay)
{
  impl_->response_delay_ms = delay.count();
}

void FakeRobStrideMotor::report_reset()
{
  impl_->mode = kMotorModeReset;
  impl_->send_feedback();
}

uint8_t FakeRobStrideMotor::mode() const {return impl_->mode.load();}
uint64_t FakeRobStrideMotor::enable_count() const {return impl_->enable_count.load();}
uint64_t FakeRobStrideMotor::motion_count() const {return impl_->motion_count.load();}
uint64_t FakeRobStrideMotor::stop_count() const {return impl_->stop_count.load();}

uint32_t FakeRobStrideMotor::parameter(uint16_t index) const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  const auto entry = impl_->parameters.find(index);
  if (entry == impl_->parameters.end()) {throw std::runtime_error("parameter was not written");}
  return entry->second;
}

}  // namespace robstride_driver::test
