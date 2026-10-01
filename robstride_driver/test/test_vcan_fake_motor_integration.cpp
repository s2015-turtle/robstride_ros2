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
#include "support/vcan_socket.hpp"

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

class ControlCycle
{
public:
  explicit ControlCycle(rs::RobStrideDriver & driver)
  : worker_([this, &driver]() {
      while (running_) {
        driver.send_commands();
        driver.update_state();
        std::this_thread::sleep_for(1ms);
      }
    }) {}
  ~ControlCycle() {stop();}
  void stop()
  {
    running_ = false;
    if (worker_.joinable()) {worker_.join();}
  }
private:
  std::atomic<bool> running_{true};
  std::thread worker_;
};

void open_and_start(rs::RobStrideDriver & driver)
{
  require(driver.open(), "driver transport did not open");
  require(driver.start(), "driver did not activate against the fake motor");
  // Enable feedback can acknowledge start before the queued neutral motion
  // reaches the bus. Wait for both responses so later assertions cannot consume
  // leftover startup feedback instead of the command under test.
  require(wait_until([&]() {
    for (const auto & motor : driver.metrics().motors) {
      if (motor.feedback_frames_received < 2) {return false;}
    }
    return true;
  }), "startup motion feedback was not received");
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
  const auto feedback_before = driver.metrics().motors[0].feedback_frames_received;
  require(driver.send_commands(), "position command submission failed");
  require(wait_until([&]() {return motor.motion_count() > motion_before;}),
    "motion command did not reach the fake motor");
  const auto commanded = motor.last_motion_frame();
  const auto expected_command = rs::make_motion_command(kMotorId, kLimits, 0.75, 0, 0, 20, 0.5);
  require(commanded && commanded->id == expected_command.id &&
    commanded->data == expected_command.data, "captured position command was incorrect");
  require(wait_until([&]() {
    return driver.metrics().motors[0].feedback_frames_received > feedback_before;
  }), "command response feedback was not received");
  require(driver.update_state(), "state update failed");
  require(std::abs(driver.joints()[0].state.position) < 0.01,
    "commanded position changed measured state without a sensor update");
  motor.report_position(0.75, kLimits);
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
           driver.metrics().motors[0].mode == rs::kMotorModeRun;
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

void test_neutral_commands_after_mode_activation()
{
  auto motor = make_fake_motor();
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_neutral_commands"));
  auto config = configuration("vcan_neutral_commands");
  auto & joint = config.joints.front();
  joint.command_limits.velocity_min = 0.0;
  joint.command_limits.effort_max = 0.0;
  joint.direction = -1.0;
  joint.gear_ratio = 2.0;
  require(driver.initialize(config), "zero-boundary configuration was rejected");
  open_and_start(driver);
  for (const auto mode : {
      rs::ClaimedInterfaces{false, true, false}, rs::ClaimedInterfaces{false, false, true}})
  {
    // Activation must discard stale nonzero commands and transmit neutral values.
    driver.joints()[0].command.velocity = 4.0;
    driver.joints()[0].command.effort = -2.0;
    require(driver.apply_command_modes({mode}), "command mode was rejected");
    const auto before = motor.motion_count();
    require(driver.send_commands(), "neutral command submission failed");
    require(wait_until([&]() {return motor.motion_count() > before;}),
      "neutral command did not reach vcan motor");
    const auto frame = motor.last_motion_frame();
    const auto expected = rs::make_motion_command(
      kMotorId, kLimits, 0.0, 0.0, 0.0, 0.0, mode.velocity ? joint.kd : 0.0);
    require(frame.has_value(), "no motion frame captured");
    // Compare encoded zero, not decoded floating point zero (16-bit quantization).
    require(frame->id == expected.id, "nonzero feedforward torque on activation");
    require(frame->data[2] == expected.data[2] && frame->data[3] == expected.data[3],
      "nonzero velocity on activation");
  }
  driver.stop();
  driver.close();
}

void test_feedback_timeout()
{
  auto motor = make_fake_motor();
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_fake_motor_timeout"));
  require(driver.initialize(configuration("vcan_fake_motor_timeout")), "initialization failed");
  open_and_start(driver);
  motor.set_feedback_enabled(false);
  require(wait_until([&]() {return driver.metrics().motors[0].feedback_stale;}),
    "diagnostics did not mark expired feedback as stale");
  require(!driver.update_state(), "feedback timeout did not fail the active driver");
  motor.set_stop_confirmation_enabled(false);
  driver.stop();
  driver.close();
}

void test_command_limits_reject_without_clamping()
{
  auto motor = make_fake_motor();
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_command_limits"));
  auto config = configuration("vcan_command_limits");
  config.joints[0].direction = -1.0;
  config.joints[0].gear_ratio = 2.0;
  config.joints[0].position_offset = 0.25;
  require(driver.initialize(config), "transformed configuration was rejected");
  open_and_start(driver);

  const rs::ClaimedInterfaces position{true, false, false};
  motor.report_position(-4.5, kLimits);  // ROS joint position = 2.5 rad.
  require(wait_until([&]() {driver.update_state(); return driver.joints()[0].state.position > 2.4;}),
    "out-of-range feedback was not received");
  const auto before_activation = motor.motion_count();
  require(!driver.apply_command_modes({position}),
    "position mode accepted out-of-range feedback");
  require(!driver.command_modes()[0].position, "rejected mode changed the claim");
  require(motor.motion_count() == before_activation, "rejected activation emitted motion");

  motor.report_position(-0.5, kLimits);  // ROS joint position = 0.5 rad.
  require(wait_until([&]() {
    driver.update_state();
    return std::abs(driver.joints()[0].state.position - 0.5) < 0.01;
  }), "in-range feedback was not received");
  require(driver.apply_command_modes({position}), "valid position mode was rejected");
  require(std::abs(driver.joints()[0].command.position - 0.5) < 0.01,
    "initial target was not the latest transformed feedback");
  const auto before_valid = motor.motion_count();
  require(driver.send_commands(), "valid position command failed");
  require(wait_until([&]() {return motor.motion_count() > before_valid;}),
    "valid position command was not transmitted");
  const auto frame = motor.last_motion_frame();
  require(frame.has_value(), "valid position frame missing");
  require(frame->data[0] == rs::encode_u16(-0.5, kLimits.position_min, kLimits.position_max) >> 8,
    "initial motor target was not the measured position");

  driver.joints()[0].command.position = 2.5;
  const auto before_invalid = motor.motion_count();
  require(!driver.send_commands(), "out-of-range position command was accepted");
  require(!driver.send_commands(), "rejected command was silently retried");
  require(motor.motion_count() == before_invalid, "rejected command was clamped and sent");
  driver.stop();
  driver.close();
}

void test_velocity_and_effort_rejection()
{
  for (const bool velocity_mode : {true, false}) {
    auto motor = make_fake_motor();
    const std::string name = velocity_mode ? "vcan_velocity_reject" : "vcan_effort_reject";
    rs::RobStrideDriver driver(rclcpp::get_logger(name));
    require(driver.initialize(configuration(name)), "command rejection setup failed");
    open_and_start(driver);
    require(driver.apply_command_modes({rs::ClaimedInterfaces{false, velocity_mode, !velocity_mode}}),
      "velocity/effort mode was rejected");
    if (velocity_mode) {
      driver.joints()[0].command.velocity = 10.1;
    } else {
      driver.joints()[0].command.effort = -3.1;
    }
    const auto before = motor.motion_count();
    require(!driver.send_commands(), "out-of-range velocity/effort was accepted");
    require(motor.motion_count() == before, "out-of-range velocity/effort was transmitted");
    driver.stop();
    driver.close();
  }
}

void test_transformed_position_boundaries()
{
  for (const double direction : {1.0, -1.0}) {
    auto motor = make_fake_motor();
    rs::RobStrideDriver driver(rclcpp::get_logger("vcan_position_boundaries"));
    auto config = configuration("vcan_position_boundaries");
    auto & joint = config.joints[0];
    joint.direction = direction;
    joint.gear_ratio = 10.0;
    joint.position_offset = 1.0;
    joint.limits.position_min = -12.566370614;
    joint.limits.position_max = 12.566370614;
    const auto range = joint.joint_position_range();
    require(range.has_value(), "position range could not be represented");
    joint.command_limits.position_min = range->first;
    joint.command_limits.position_max = range->second;
    joint.command_limits.velocity_min = -1.0;
    joint.command_limits.velocity_max = 1.0;
    require(driver.initialize(config), "boundary configuration failed");
    open_and_start(driver);
    require(driver.apply_command_modes({rs::ClaimedInterfaces{true, false, false}}),
      "position mode activation failed");
    for (const double endpoint : {range->first, range->second}) {
      driver.joints()[0].command.position = endpoint;
      const auto expected = rs::make_motion_command(
        kMotorId, joint.limits, *joint.checked_joint_to_motor_position(endpoint),
        0.0, 0.0, joint.kp, joint.kd);
      require(driver.send_commands(), "valid boundary latched a command rejection");
      require(wait_until([&]() {
        const auto frame = motor.last_motion_frame();
        return frame && frame->id == expected.id && frame->data == expected.data;
      }), "boundary frame did not reach fake motor");
    }
    // A subsequent ordinary command confirms neither endpoint latched the batch error.
    driver.joints()[0].command.position = joint.position_offset;
    require(driver.send_commands(), "boundary command left the driver rejected");
    driver.stop();
    driver.close();
  }
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
  require(!driver.start(), "missing parameter confirmation did not reject activation");
  require(motor.parameter(rs::kIndexCanTimeout) == 4000,
    "activation failed before the parameter request reached the motor");
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

void test_configured_watchdog_deadline()
{
  auto motor = make_fake_motor();
  test::VcanSocket host("vcan0");
  const auto send = [&](const rs::Frame & frame) {host.send(frame.id, frame.data);};
  // Nonoverlapping [200, 700) ms and [1000, 1500) ms windows catch a
  // hard-coded timeout while retaining a 500 ms scheduling allowance.
  for (const uint32_t ticks : {4000u, 20000u}) {
    send(rs::make_write_u32(kMotorId, kHostId, rs::kIndexCanTimeout, ticks));
    const auto enabled_before = motor.enable_count();
    send(rs::make_enable(kMotorId, kHostId));
    require(wait_until([&]() {return motor.enable_count() > enabled_before;}),
      "raw CAN enable was not observed");
    const auto motion_before = motor.motion_count();
    const auto sent_at = std::chrono::steady_clock::now();
    send(rs::make_motion_command(kMotorId, kLimits, 1.0, 0, 0, 0, 0));
    require(wait_until([&]() {return motor.motion_count() > motion_before;}),
      "raw CAN watchdog-refresh command was not observed");
    const auto observed_at = std::chrono::steady_clock::now();
    require(wait_until([&]() {
      motor.check_worker();
      // Neither reads addressed to this motor nor motion to another motor
      // refreshes this fake's command watchdog.
      send(rs::make_read_parameter(kMotorId, kHostId, rs::kIndexCanTimeout));
      send(rs::make_motion_command(7, kLimits, 0, 0, 0, 0, 0));
      return motor.mode() == rs::kMotorModeReset;
    }), "watchdog was refreshed by parameter reads or another motor ID");
    const auto reset_at = std::chrono::steady_clock::now();
    const auto configured_timeout = std::chrono::microseconds(static_cast<int64_t>(ticks) * 50);
    require(reset_at - sent_at >= configured_timeout,
      "watchdog expired before its configured monotonic deadline");
    require(reset_at - observed_at < configured_timeout + 500ms,
      "watchdog exceeded its bounded scheduling allowance");
    require(motor.parameter(rs::kIndexCanTimeout) == ticks,
      "fake did not retain the configured watchdog parameter");
    const auto after_reset = motor.motion_count();
    send(rs::make_motion_command(kMotorId, kLimits, 0, 0, 0, 0, 0));
    require(wait_until([&]() {return motor.motion_count() > after_reset;}),
      "commands did not resume after watchdog expiry");
    require(motor.mode() == rs::kMotorModeReset,
      "motion command implicitly re-enabled a watchdog-reset motor");
  }
  require(motor.watchdog_reset_count() == 2, "watchdog reset count was not one per expiry");
}

void test_startup_outside_operational_limits()
{
  auto motor = make_fake_motor();
  motor.report_position(-4.5, kLimits);  // transformed joint position = 2.5, limit = 2.
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_outside_startup"));
  auto config = configuration("vcan_outside_startup");
  config.joints[0].direction = -1.0;
  config.joints[0].gear_ratio = 2.0;
  config.joints[0].position_offset = 0.25;
  require(driver.initialize(config), "outside-startup configuration failed");
  open_and_start(driver);
  require(wait_until([&]() {return motor.motion_count() > 0;}),
    "startup zero-gain motion frame was not observed");
  const auto startup = motor.last_motion_frame();
  const auto neutral = rs::make_motion_command(kMotorId, kLimits, 0, 0, 0, 0, 0);
  require(startup && startup->id == neutral.id && startup->data == neutral.data,
    "startup was not a zero-gain neutral command");
  require(driver.update_state(), "outside-limit measured state failed read");
  require(std::abs(driver.joints()[0].state.position - 2.5) < 0.01,
    "zero-gain startup teleported measured state into operational limits");
  require(!driver.apply_command_modes({rs::ClaimedInterfaces{true, false, false}}),
    "out-of-limit initial position unexpectedly activated position control");
  require(!driver.command_modes()[0].position, "failed activation retained position claim");
  motor.report_position(-0.5, kLimits);
  require(wait_until([&]() {
    require(driver.update_state(), "communication did not resume after rejected activation");
    return std::abs(driver.joints()[0].state.position - 0.5) < 0.01;
  }), "restored measured feedback was not routed");
  require(driver.apply_command_modes({rs::ClaimedInterfaces{true, false, false}}),
    "position activation did not recover with in-range measured feedback");
  const auto before = motor.motion_count();
  require(driver.send_commands(), "valid command failed after rejected activation");
  require(wait_until([&]() {return motor.motion_count() > before;}),
    "communication did not resume after rejected activation");
  motor.check_worker();
}

void test_two_motor_fault_isolation(bool fail_on_timeout)
{
  auto first = make_fake_motor();
  test::FakeRobStrideMotor second(test::FakeMotorOptions{"vcan0", 7, kHostId});
  first.report_position(0.6, kLimits);
  second.report_position(-0.8, kLimits);
  rs::RobStrideDriver driver(rclcpp::get_logger("vcan_two_motor_faults"));
  auto config = configuration("vcan_two_motor_faults");
  config.settings.fail_on_feedback_timeout = fail_on_timeout;
  auto second_joint = config.joints[0];
  second_joint.name = "second_fake_joint";
  second_joint.can_id = 7;
  config.joints.push_back(second_joint);
  require(driver.initialize(config), "two-motor configuration failed");
  open_and_start(driver);
  require(driver.update_state(), "initial two-motor read failed");
  require(std::abs(driver.joints()[0].state.position - 0.6) < 0.01 &&
    std::abs(driver.joints()[1].state.position + 0.8) < 0.01,
    "measured feedback was routed to the wrong motor");
  const auto cycle = [&]() {
      first.check_worker();
      second.check_worker();
      require(driver.send_commands(), "two-motor command batch failed");
    };

  first.set_feedback_enabled(false);
  const auto second_feedback_before = driver.metrics().motors[1].feedback_frames_received;
  require(wait_until([&]() {
    cycle();
    return driver.metrics().motors[0].feedback_stale;
  }), "single-motor feedback did not expire");
  const auto stale = driver.metrics();
  require(stale.motors[0].can_id == 1 && stale.motors[1].can_id == 7,
    "diagnostics lost motor ID routing");
  require(!stale.motors[1].feedback_stale &&
    stale.motors[1].feedback_frames_received > second_feedback_before,
    "one motor's feedback loss contaminated the healthy motor");
  require(driver.update_state() == !fail_on_timeout,
    "whole-component read result ignored configured feedback timeout policy");
  first.set_feedback_enabled(true);
  require(wait_until([&]() {
    cycle();
    return !driver.metrics().motors[0].feedback_stale && driver.update_state();
  }), "reads did not recover after feedback resumed");

  const auto first_enables = first.enable_count();
  const auto second_enables = second.enable_count();
  first.report_reset();
  require(wait_until([&]() {
    cycle();
    require(driver.update_state(), "single-motor Reset failed before recovery deadline");
    const auto metrics = driver.metrics();
    return first.enable_count() > first_enables &&
           metrics.motors[0].mode == rs::kMotorModeRun && !metrics.motors[0].recovery_active;
  }), "single-motor Reset was not recovered");
  require(second.enable_count() == second_enables &&
    driver.metrics().motors[1].recovery_attempts == 0,
    "recovery enable was routed to the healthy motor");

  // Lose commands to just ID 1. ID 7 keeps receiving host traffic on the same bus.
  const auto first_resets = first.watchdog_reset_count();
  const auto second_resets = second.watchdog_reset_count();
  const auto second_motion = second.motion_count();
  first.set_commands_enabled(false);
  require(wait_until([&]() {
    cycle();
    return first.watchdog_reset_count() > first_resets;
  }), "configured watchdog did not reset the motor after command loss");
  require(first.mode() == rs::kMotorModeReset && second.mode() == rs::kMotorModeRun &&
    second.watchdog_reset_count() == second_resets && second.motion_count() > second_motion,
    "unrelated motor traffic refreshed the lost motor watchdog or reset the healthy motor");
  first.set_commands_enabled(true);
  require(wait_until([&]() {
    cycle();
    // A stale read may precede delivery of the watchdog Reset feedback.
    driver.update_state();
    return driver.metrics().motors[0].mode == rs::kMotorModeRun &&
           !driver.metrics().motors[0].recovery_active &&
           first.mode() == rs::kMotorModeRun;
  }), "watchdog-reset motor did not recover after commands resumed");
  require(second.enable_count() == second_enables,
    "watchdog recovery enabled the wrong motor");
  first.check_worker();
  second.check_worker();
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

  ControlCycle control_cycle(driver);

  const uint64_t enables_before = motor.enable_count();
  motor.report_reset();
  const bool recovery_observed =
    wait_until([&]() {return motor.enable_count() > enables_before;});
  driver.stop();
  control_cycle.stop();
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
    test_configured_watchdog_deadline();
    test_startup_outside_operational_limits();
    test_two_motor_fault_isolation(true);
    test_two_motor_fault_isolation(false);
    test_complete_lifecycle_and_recovery();
    test_neutral_commands_after_mode_activation();
    test_feedback_timeout();
    test_command_limits_reject_without_clamping();
    test_velocity_and_effort_rejection();
    test_transformed_position_boundaries();
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
