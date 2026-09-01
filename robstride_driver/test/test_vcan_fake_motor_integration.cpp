#include <linux/can.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "robstride_driver/driver.hpp"
#include "robstride_driver/protocol.hpp"

using namespace std::chrono_literals;
namespace rs = robstride_driver;

namespace
{
constexpr uint8_t kMotorId = 1;
constexpr uint8_t kHostId = 0xfd;
constexpr double kPi = 3.14159265358979323846;
constexpr rs::Limits kLimits{
  -4.0 * kPi, 4.0 * kPi, -50.0, 50.0, -6.0, 6.0, -6.0, 6.0, 500.0, 5.0};

void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

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

class CanSocket
{
public:
  explicit CanSocket(const char * interface_name)
  {
    descriptor_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (descriptor_ < 0) {throw std::runtime_error("could not create CAN socket");}

    ifreq request{};
    std::strncpy(request.ifr_name, interface_name, IFNAMSIZ - 1);
    if (ioctl(descriptor_, SIOCGIFINDEX, &request) < 0) {
      close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error("could not resolve vcan interface");
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(descriptor_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error("could not bind CAN socket");
    }
  }

  ~CanSocket()
  {
    if (descriptor_ >= 0) {close(descriptor_);}
  }

  bool receive(can_frame & frame, std::chrono::milliseconds timeout)
  {
    pollfd event{descriptor_, POLLIN, 0};
    const int result = poll(&event, 1, static_cast<int>(timeout.count()));
    if (result == 0) {return false;}
    if (result < 0) {throw std::runtime_error("could not poll virtual CAN interface");}
    if (read(descriptor_, &frame, sizeof(frame)) != sizeof(frame)) {
      throw std::runtime_error("could not read virtual CAN frame");
    }
    return true;
  }

  void send(uint32_t id, const std::array<uint8_t, 8> & data)
  {
    can_frame frame{};
    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::copy(data.begin(), data.end(), frame.data);
    if (write(descriptor_, &frame, sizeof(frame)) != sizeof(frame)) {
      throw std::runtime_error("could not send virtual CAN frame");
    }
  }

private:
  int descriptor_{-1};
};

class FakeRobStrideMotor
{
public:
  FakeRobStrideMotor()
  : socket_("vcan0"), worker_([this]() {run();})
  {
  }

  ~FakeRobStrideMotor()
  {
    running_ = false;
    if (worker_.joinable()) {worker_.join();}
  }

  FakeRobStrideMotor(const FakeRobStrideMotor &) = delete;
  FakeRobStrideMotor & operator=(const FakeRobStrideMotor &) = delete;

  void set_feedback_enabled(bool enabled) {feedback_enabled_ = enabled;}
  void set_parameter_confirmation_enabled(bool enabled)
  {
    parameter_confirmation_enabled_ = enabled;
  }
  void set_stop_confirmation_enabled(bool enabled) {stop_confirmation_enabled_ = enabled;}
  void set_response_delay(std::chrono::milliseconds delay) {response_delay_ms_ = delay.count();}

  void report_reset()
  {
    mode_ = rs::kMotorModeReset;
    send_feedback();
  }

  uint8_t mode() const {return mode_.load();}
  uint64_t enable_count() const {return enable_count_.load();}
  uint64_t motion_count() const {return motion_count_.load();}
  uint64_t stop_count() const {return stop_count_.load();}

  uint32_t parameter(uint16_t index) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto entry = parameters_.find(index);
    if (entry == parameters_.end()) {throw std::runtime_error("parameter was not written");}
    return entry->second;
  }

private:
  void run()
  {
    try {
      while (running_) {
        can_frame raw{};
        if (!socket_.receive(raw, 20ms)) {continue;}
        if ((raw.can_id & CAN_EFF_FLAG) == 0 || raw.can_dlc != 8) {continue;}
        handle(raw.can_id & CAN_EFF_MASK, raw.data);
      }
    } catch (const std::exception & error) {
      worker_error_ = error.what();
      running_ = false;
    }
  }

  void handle(uint32_t id, const uint8_t * raw_data)
  {
    const uint8_t type = static_cast<uint8_t>((id >> 24) & 0x1f);
    const uint8_t destination = static_cast<uint8_t>(id & 0xff);
    if (destination != kMotorId) {return;}

    std::array<uint8_t, 8> data{};
    std::copy(raw_data, raw_data + data.size(), data.begin());
    if (type == rs::kTypeWriteParameter) {
      const uint16_t index = static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
      const uint32_t value = static_cast<uint32_t>(data[4]) |
        (static_cast<uint32_t>(data[5]) << 8) |
        (static_cast<uint32_t>(data[6]) << 16) |
        (static_cast<uint32_t>(data[7]) << 24);
      std::lock_guard<std::mutex> lock(state_mutex_);
      parameters_[index] = value;
    } else if (type == rs::kTypeReadParameter) {
      if (!parameter_confirmation_enabled_) {return;}
      const uint16_t index = static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8);
      uint32_t value = 0;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto entry = parameters_.find(index);
        if (entry != parameters_.end()) {value = entry->second;}
      }
      send_parameter(index, value);
    } else if (type == rs::kTypeEnable) {
      ++enable_count_;
      mode_ = rs::kMotorModeRun;
      send_feedback();
    } else if (type == rs::kTypeStop) {
      ++stop_count_;
      mode_ = rs::kMotorModeReset;
      if (stop_confirmation_enabled_) {send_feedback();}
    } else if (type == rs::kTypeMotionControl) {
      ++motion_count_;
      position_raw_ = read_be16(data, 0);
      velocity_raw_ = read_be16(data, 2);
      effort_raw_ = static_cast<uint16_t>((id >> 8) & 0xffff);
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
      (static_cast<uint32_t>(rs::kTypeReadParameter) << 24) |
      (static_cast<uint32_t>(kMotorId) << 8) | kHostId;
    socket_.send(id, data);
  }

  void send_feedback()
  {
    if (!feedback_enabled_) {return;}
    delay_response();
    std::array<uint8_t, 8> data{};
    put_be16(data, 0, position_raw_.load());
    put_be16(data, 2, velocity_raw_.load());
    put_be16(data, 4, effort_raw_.load());
    put_be16(data, 6, 300);
    const uint16_t area =
      (static_cast<uint16_t>(mode_.load() & 0x03) << 14) | kMotorId;
    const uint32_t id =
      (static_cast<uint32_t>(rs::kTypeFeedback) << 24) |
      (static_cast<uint32_t>(area) << 8) | kHostId;
    socket_.send(id, data);
  }

  void delay_response() const
  {
    const auto delay = response_delay_ms_.load();
    if (delay > 0) {std::this_thread::sleep_for(std::chrono::milliseconds(delay));}
  }

  CanSocket socket_;
  std::atomic<bool> running_{true};
  std::atomic<bool> feedback_enabled_{true};
  std::atomic<bool> parameter_confirmation_enabled_{true};
  std::atomic<bool> stop_confirmation_enabled_{true};
  std::atomic<int64_t> response_delay_ms_{0};
  std::atomic<uint8_t> mode_{rs::kMotorModeReset};
  std::atomic<uint16_t> position_raw_{32768};
  std::atomic<uint16_t> velocity_raw_{32768};
  std::atomic<uint16_t> effort_raw_{32768};
  std::atomic<uint64_t> enable_count_{0};
  std::atomic<uint64_t> motion_count_{0};
  std::atomic<uint64_t> stop_count_{0};
  mutable std::mutex state_mutex_;
  std::unordered_map<uint16_t, uint32_t> parameters_;
  std::string worker_error_;
  std::thread worker_;
};

rs::DriverConfiguration configuration(const std::string & node_name)
{
  rs::DriverConfiguration configuration;
  configuration.settings.host_id = kHostId;
  configuration.settings.transport.node_name = node_name;
  configuration.settings.transport.transmit_topic = "/robstride_vcan/to_bus";
  configuration.settings.transport.receive_topic = "/robstride_vcan/from_bus";
  configuration.settings.connection_timeout = 5s;
  configuration.settings.startup_confirmation_timeout = 500ms;
  configuration.settings.startup_retries = 2;
  configuration.settings.feedback_timeout = 250ms;
  configuration.settings.recovery_timeout = 800ms;
  configuration.settings.recovery_retry_interval = 50ms;
  configuration.settings.stop_repetitions = 2;
  configuration.settings.stop_interval = 5ms;
  configuration.settings.stop_confirmation_timeout = 300ms;
  configuration.settings.clear_faults_on_start = false;

  rs::JointData joint;
  joint.name = "fake_motor_joint";
  joint.can_id = kMotorId;
  joint.limits = kLimits;
  joint.kp = 20.0;
  joint.kd = 0.5;
  joint.can_timeout_ticks = 4000;
  joint.command_limits = {-2.0, 2.0, -10.0, 10.0, -3.0, 3.0};
  configuration.joints.push_back(joint);
  return configuration;
}

template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

void open_and_start(rs::RobStrideDriver & driver)
{
  require(driver.open(), "driver transport did not open");
  std::this_thread::sleep_for(500ms);
  require(driver.start(), "driver did not activate against the fake motor");
}

void test_complete_lifecycle_and_recovery()
{
  FakeRobStrideMotor motor;
  motor.set_response_delay(5ms);
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_lifecycle"));
  require(driver.initialize(configuration("vcan_fake_motor_lifecycle")), "initialization failed");
  open_and_start(driver);

  require(motor.parameter(rs::kIndexCanTimeout) == 4000, "watchdog was not configured");
  require(motor.parameter(rs::kIndexRunMode) == 0, "motion mode was not configured");
  require(motor.mode() == rs::kMotorModeRun, "motor was not enabled");

  require(driver.apply_command_modes({rs::ClaimedInterfaces{true, false, false}}),
    "position command mode was rejected");
  driver.joints()[0].command.position = 0.75;
  const uint64_t motion_before = motor.motion_count();
  driver.send_commands();
  require(wait_until([&]() {return motor.motion_count() > motion_before;}),
    "motion command did not reach the fake motor");
  require(wait_until([&]() {
    driver.update_state();
    return std::abs(driver.joints()[0].state.position - 0.75) < 0.01;
  }), "simulated position feedback did not reach the driver");

  const uint64_t enables_before = motor.enable_count();
  motor.report_reset();
  require(wait_until([&]() {
    driver.update_state();
    return motor.enable_count() > enables_before;
  }), "unexpected Reset mode did not trigger an enable retry");
  require(wait_until([&]() {
    return driver.update_state() && motor.mode() == rs::kMotorModeRun &&
           driver.joints()[0].feedback_status.mode == rs::kMotorModeRun;
  }),
    "motor did not recover to Run mode");

  driver.stop();
  require(wait_until([&]() {return motor.mode() == rs::kMotorModeReset;}),
    "motor did not confirm Reset during deactivation");
  require(motor.stop_count() >= 2, "configured stop repetitions were not sent");
  driver.close();
}

void test_feedback_timeout()
{
  FakeRobStrideMotor motor;
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_timeout"));
  require(driver.initialize(configuration("vcan_fake_motor_timeout")), "initialization failed");
  open_and_start(driver);
  motor.set_feedback_enabled(false);
  std::this_thread::sleep_for(300ms);
  require(!driver.update_state(), "feedback timeout did not fail the active driver");
  motor.set_stop_confirmation_enabled(false);
  driver.stop();
  driver.close();
}

void test_parameter_and_stop_confirmation_failures()
{
  {
    FakeRobStrideMotor motor;
    motor.set_parameter_confirmation_enabled(false);
    rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_parameter_failure"));
    auto config = configuration("vcan_fake_motor_parameter_failure");
    config.settings.startup_confirmation_timeout = 50ms;
    config.settings.startup_retries = 1;
    require(driver.initialize(std::move(config)), "initialization failed");
    require(driver.open(), "driver transport did not open");
    std::this_thread::sleep_for(500ms);
    require(!driver.start(), "missing parameter confirmation did not reject activation");
    driver.close();
  }

  {
    FakeRobStrideMotor motor;
    rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_stop_failure"));
    require(driver.initialize(configuration("vcan_fake_motor_stop_failure")),
      "initialization failed");
    open_and_start(driver);
    motor.set_stop_confirmation_enabled(false);
    const auto started = std::chrono::steady_clock::now();
    driver.stop();
    const auto elapsed = std::chrono::steady_clock::now() - started;
    require(elapsed < 1s, "missing stop confirmation blocked shutdown");
    require(motor.stop_count() >= 2, "stop retries were not sent without confirmation");
    driver.close();
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    test_complete_lifecycle_and_recovery();
    test_feedback_timeout();
    test_parameter_and_stop_confirmation_failures();
    rclcpp::shutdown();
    std::cout << "vcan fake RobStride motor integration passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "vcan fake RobStride motor integration failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
