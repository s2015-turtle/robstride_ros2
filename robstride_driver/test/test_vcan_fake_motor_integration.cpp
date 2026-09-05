#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "robstride_driver/driver.hpp"
#include "robstride_driver/protocol.hpp"
#include "support/fake_robstride_motor.hpp"

using namespace std::chrono_literals;
namespace rs = robstride_driver;
namespace test = robstride_driver::test;

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

test::FakeRobStrideMotor make_fake_motor()
{
  return test::FakeRobStrideMotor(test::FakeMotorOptions{"vcan0", kMotorId, kHostId});
}

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
  auto motor = make_fake_motor();
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
  const auto healthy_metrics = driver.metrics();
  require(healthy_metrics.hardware_active, "diagnostics did not report active hardware");
  require(healthy_metrics.motors.size() == 1, "diagnostics motor count is incorrect");
  require(healthy_metrics.motors[0].mode == rs::kMotorModeRun,
    "diagnostics did not report Run mode");
  require(std::abs(healthy_metrics.motors[0].temperature - 30.0) < 0.01,
    "diagnostics did not report motor temperature");
  require(!healthy_metrics.motors[0].feedback_stale,
    "current motor feedback was reported as stale");

  const uint64_t enables_before = motor.enable_count();
  motor.report_reset();
  require(wait_until([&]() {
    driver.update_state();
    return motor.enable_count() > enables_before;
  }), "unexpected Reset mode did not trigger an enable retry");
  require(wait_until([&]() {
    return driver.update_state() && motor.mode() == rs::kMotorModeRun &&
           driver.joints()[0].feedback_status.mode == rs::kMotorModeRun;
  }), "motor did not recover to Run mode");
  const auto recovered_metrics = driver.metrics();
  require(recovered_metrics.motors[0].recovery_attempts > 0,
    "diagnostics did not retain the recovery attempt count");
  require(!recovered_metrics.motors[0].recovery_active,
    "diagnostics still reported recovery after Run mode returned");

  driver.stop();
  require(wait_until([&]() {return motor.mode() == rs::kMotorModeReset;}),
    "motor did not confirm Reset during deactivation");
  // DDS acknowledgement and the first Reset feedback can precede delivery of
  // the final stop frame through the bridge to the fake motor's receive thread.
  require(wait_until([&]() {return motor.stop_count() >= 2;}),
    "configured stop repetitions were not sent");
  driver.close();
}

void test_feedback_timeout()
{
  auto motor = make_fake_motor();
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_timeout"));
  require(driver.initialize(configuration("vcan_fake_motor_timeout")), "initialization failed");
  open_and_start(driver);
  motor.set_feedback_enabled(false);
  std::this_thread::sleep_for(300ms);
  require(driver.metrics().motors[0].feedback_stale,
    "diagnostics did not mark expired feedback as stale");
  require(!driver.update_state(), "feedback timeout did not fail the active driver");
  motor.set_stop_confirmation_enabled(false);
  driver.stop();
  driver.close();
}

void test_parameter_confirmation_failure()
{
  auto motor = make_fake_motor();
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

void test_missing_stop_confirmation()
{
  auto motor = make_fake_motor();
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_stop_failure"));
  require(driver.initialize(configuration("vcan_fake_motor_stop_failure")),
    "initialization failed");
  open_and_start(driver);
  motor.set_stop_confirmation_enabled(false);
  const auto started = std::chrono::steady_clock::now();
  driver.stop();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  require(elapsed < 1s, "missing stop confirmation blocked shutdown");
  require(wait_until([&]() {return motor.stop_count() >= 2;}),
    "stop retries were not sent without confirmation");
  driver.close();
}

void test_concurrent_control_recovery_and_shutdown()
{
  auto motor = make_fake_motor();
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_concurrency"));
  require(driver.initialize(configuration("vcan_fake_motor_concurrency")),
    "initialization failed");
  open_and_start(driver);
  require(driver.apply_command_modes({rs::ClaimedInterfaces{false, true, false}}),
    "velocity command mode was rejected");
  driver.joints()[0].command.velocity = 1.0;

  std::atomic<bool> cycling{true};
  std::thread control_cycle([&]() {
      while (cycling) {
        driver.send_commands();
        driver.update_state();
        std::this_thread::sleep_for(1ms);
      }
    });

  const uint64_t enables_before = motor.enable_count();
  motor.report_reset();
  const bool recovery_observed =
    wait_until([&]() {return motor.enable_count() > enables_before;});
  driver.stop();
  cycling = false;
  control_cycle.join();
  require(recovery_observed, "concurrent feedback did not trigger recovery");
  require(wait_until([&]() {return motor.mode() == rs::kMotorModeReset;}),
    "concurrent shutdown did not stop the motor");
  driver.close();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    test_complete_lifecycle_and_recovery();
    test_feedback_timeout();
    test_parameter_confirmation_failure();
    test_missing_stop_confirmation();
    test_concurrent_control_recovery_and_shutdown();
    rclcpp::shutdown();
    std::cout << "vcan fake RobStride motor integration passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "vcan fake RobStride motor integration failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
