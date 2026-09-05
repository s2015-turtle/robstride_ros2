#include "robstride_driver/driver.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include <rclcpp/logging.hpp>

#include "robstride_driver/protocol.hpp"

namespace robstride_driver
{
namespace
{
int64_t steady_time_ns(std::chrono::steady_clock::time_point time) noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

double rate_hz(uint64_t count, std::chrono::nanoseconds period) noexcept
{
  const double seconds = std::chrono::duration<double>(period).count();
  return seconds > 0.0 ? static_cast<double>(count) / seconds : 0.0;
}
}  // namespace

RobStrideDriver::RobStrideDriver(rclcpp::Logger logger)
: logger_(std::move(logger)), log_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
{
}

RobStrideDriver::~RobStrideDriver() noexcept
{
  stop();
  close();
}

bool RobStrideDriver::initialize(DriverConfiguration configuration)
{
  if (configuration.joints.empty()) {
    RCLCPP_ERROR(logger_, "Driver configuration must contain at least one joint");
    return false;
  }
  if (configuration.settings.transport.node_name.empty() ||
    configuration.settings.transport.receive_qos_depth == 0)
  {
    RCLCPP_ERROR(logger_, "Driver transport configuration is invalid");
    return false;
  }
  std::set<uint8_t> can_ids;
  for (const auto & joint : configuration.joints) {
    const auto & command = joint.command_limits;
    if (joint.can_id == 0 || joint.can_timeout_ticks == 0 ||
      !can_ids.insert(joint.can_id).second ||
      !std::isfinite(command.position_min) || !std::isfinite(command.position_max) ||
      !std::isfinite(command.velocity_min) || !std::isfinite(command.velocity_max) ||
      !std::isfinite(command.effort_min) || !std::isfinite(command.effort_max) ||
      !(command.position_min < command.position_max) ||
      !(command.velocity_min < command.velocity_max) ||
      !(command.effort_min < command.effort_max))
    {
      RCLCPP_ERROR(
        logger_, "Joint '%s' has an invalid ID, watchdog, or command limit", joint.name.c_str());
      return false;
    }
  }
  configuration.settings.transport.motor_count = configuration.joints.size();
  settings_ = std::move(configuration.settings);
  joints_ = std::move(configuration.joints);
  can_id_to_joint_.clear();
  for (size_t index = 0; index < joints_.size(); ++index) {
    can_id_to_joint_[joints_[index].can_id] = index;
  }
  runtime_events_.clear();
  runtime_events_.reserve(joints_.size());
  command_snapshot_.resize(joints_.size());
  recovery_updates_.clear();
  recovery_updates_.reserve(joints_.size());
  for (size_t index = 0; index < joints_.size(); ++index) {
    command_snapshot_[index].motor_index = index;
  }
  feedback_metrics_ = std::make_unique<AtomicMotorFeedback[]>(joints_.size());
  recovery_metrics_ = std::make_unique<AtomicRecoveryMetrics[]>(joints_.size());
  reset_metrics();
  return true;
}

std::vector<JointData> & RobStrideDriver::joints() noexcept
{
  return joints_;
}

bool RobStrideDriver::open()
{
  try {
    reset_metrics();
    transport_ = std::make_unique<CanTransport>(
      settings_.transport,
      [this](can_msgs::msg::Frame::ConstSharedPtr frame) {receive_frame(std::move(frame));},
      CanTransport::FrameSink{}, [this]() {return metrics();});
    transport_->start();
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "CAN topic setup failed: %s", error.what());
    close();
    return false;
  }
}

void RobStrideDriver::close() noexcept
{
  active_ = false;
  try {
    if (transport_) {transport_->stop();}
  } catch (...) {}
  transport_.reset();
}

bool RobStrideDriver::start()
{
  if (!transport_ || !transport_->wait_for_endpoints(settings_.connection_timeout)) {
    RCLCPP_ERROR(
      logger_, "Timed out waiting for CAN bridge endpoints on '%s' and '%s'",
      settings_.transport.transmit_topic.c_str(), settings_.transport.receive_topic.c_str());
    return false;
  }

  for (auto & joint : joints_) {
    if (settings_.clear_faults_on_start) {
      transport_->send_transaction(make_stop(joint.can_id, settings_.host_id, true));
    }
    if (!write_and_confirm_parameter(joint, kIndexCanTimeout, joint.can_timeout_ticks) ||
      !write_and_confirm_parameter(joint, kIndexRunMode, 0))
    {
      disable_all();
      return false;
    }
    if (settings_.set_zero_on_start) {
      transport_->send_transaction(make_set_zero(joint.can_id, settings_.host_id));
    }
  }

  if (!enable_and_confirm_all()) {
    disable_all();
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto & joint : joints_) {
      joint.state = joint.feedback;
      joint.recovery = RecoveryState{};
    }
  }
  transport_->enable_active_commands();
  activated_at_ = std::chrono::steady_clock::now();
  active_ = true;
  return true;
}

void RobStrideDriver::stop() noexcept
{
  try {disable_all();} catch (...) {active_ = false;}
  if (recovery_metrics_) {
    for (size_t index = 0; index < joints_.size(); ++index) {
      recovery_metrics_[index].active = false;
    }
  }
}

bool RobStrideDriver::update_state()
{
  if (!active_) {return true;}
  const auto now = std::chrono::steady_clock::now();
  bool read_failed = false;
  runtime_events_.clear();
  recovery_updates_.clear();
  {
    // The controller-manager path only snapshots driver state while holding state_mutex_.
    // Transport queue operations happen after this scope, so driver and transport locks are
    // never nested. Both vectors reserve one entry per joint during initialize().
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t joint_index = 0; joint_index < joints_.size(); ++joint_index) {
      auto & joint = joints_[joint_index];
      if (joint.feedback_status.received) {joint.state = joint.feedback;}

      if (settings_.fail_on_feedback_timeout) {
        const auto reference =
          joint.feedback_status.received ? joint.feedback_status.timestamp : activated_at_;
        if (now - reference > settings_.feedback_timeout) {
          runtime_events_.push_back(
            RuntimeEvent{RuntimeEventKind::feedback_timeout, joint_index});
          read_failed = true;
          break;
        }
      }

      if (!joint.feedback_status.received) {continue;}
      const int attempts_before_update = joint.recovery.attempts;
      const auto action = joint.recovery.update(
        joint.feedback_status.mode, now, settings_.recovery_timeout,
        settings_.recovery_retry_interval);
      recovery_metrics_[joint_index].active = joint.recovery.active;
      if (joint.recovery.attempts > attempts_before_update) {
        recovery_metrics_[joint_index].attempts.fetch_add(
          static_cast<uint64_t>(joint.recovery.attempts - attempts_before_update));
      }
      if (action == RecoveryAction::recovered) {
        recovery_updates_.push_back(
          CanTransport::RecoveryUpdate{joint_index, std::nullopt});
        runtime_events_.push_back(
          RuntimeEvent{
            RuntimeEventKind::recovered, joint_index, kMotorModeRun, attempts_before_update});
      } else if (action == RecoveryAction::failed) {
        runtime_events_.push_back(
          RuntimeEvent{
            RuntimeEventKind::recovery_failed, joint_index, joint.recovery.detected_mode,
            joint.recovery.attempts});
        read_failed = true;
        break;
      } else if (action == RecoveryAction::send_enable) {
        if (joint.recovery.attempts == 1) {
          runtime_events_.push_back(
            RuntimeEvent{
              RuntimeEventKind::recovery_started, joint_index, joint.recovery.detected_mode,
              joint.recovery.attempts});
        }
        recovery_updates_.push_back(
          CanTransport::RecoveryUpdate{
            joint_index, make_enable(joint.can_id, settings_.host_id)});
      }
    }
  }
  transport_->apply_recovery_updates(recovery_updates_);
  log_runtime_events();
  return !read_failed;
}

bool RobStrideDriver::send_commands()
{
  if (!active_) {return true;}
  {
    // command_snapshot_ is fixed-size after initialize(). The ros2_control write callback is its
    // sole producer; state_mutex_ is released before the batch takes the transport queue lock.
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!active_) {return true;}
    for (size_t joint_index = 0; joint_index < joints_.size(); ++joint_index) {
      const auto & joint = joints_[joint_index];
      const double joint_position =
        joint.claimed.position && std::isfinite(joint.command.position) ?
        joint.command_limits.clamp_position(joint.command.position) :
        joint.command_limits.clamp_position(joint.state.position);
      const double motor_position = std::isfinite(joint_position) ?
        joint.direction * (joint_position - joint.position_offset) * joint.gear_ratio : 0.0;
      const double motor_velocity =
        joint.claimed.velocity && std::isfinite(joint.command.velocity) ?
        joint.direction * joint.command_limits.clamp_velocity(joint.command.velocity) *
        joint.gear_ratio : 0.0;
      const double motor_effort =
        joint.claimed.effort && std::isfinite(joint.command.effort) ?
        joint.joint_to_motor_effort(joint.command_limits.clamp_effort(joint.command.effort)) :
        0.0;
      const double kp = joint.claimed.position ? joint.kp : 0.0;
      const double kd = (joint.claimed.position || joint.claimed.velocity) ? joint.kd : 0.0;
      command_snapshot_[joint_index].frame = make_motion_command(
        joint.can_id, joint.limits, motor_position, motor_velocity, motor_effort, kp, kd);
    }
  }
  transport_->queue_motion_frames(command_snapshot_);
  return check_transport_health();
}

bool RobStrideDriver::check_transport_health()
{
  if (!transport_) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *log_clock_, 1000, "CAN transport is unavailable while hardware is active");
    return false;
  }

  const auto health = transport_->health(settings_.transmit_failure_timeout);
  if (health.state == CanTransportHealthState::healthy) {return true;}

  const auto duration_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(health.duration).count();
  const char * condition = "unknown transport failure";
  if (health.state == CanTransportHealthState::bridge_unavailable) {
    condition = "CAN bridge transmit endpoint is unavailable";
  } else if (health.state == CanTransportHealthState::transmit_stalled) {
    condition = "CAN transmit worker is not making progress";
  } else if (health.state == CanTransportHealthState::worker_stopped) {
    condition = "CAN transmit worker stopped unexpectedly";
  }

  if (!health.persistent) {
    RCLCPP_WARN_THROTTLE(
      logger_, *log_clock_, 1000, "%s (%lld ms); waiting up to %lld ms before returning ERROR",
      condition, static_cast<long long>(duration_ms),
      static_cast<long long>(settings_.transmit_failure_timeout.count()));
    return true;
  }
  RCLCPP_ERROR_THROTTLE(
    logger_, *log_clock_, 1000, "%s for %lld ms; returning ERROR to ros2_control",
    condition, static_cast<long long>(duration_ms));
  return false;
}

std::vector<ClaimedInterfaces> RobStrideDriver::command_modes() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<ClaimedInterfaces> modes;
  modes.reserve(joints_.size());
  for (const auto & joint : joints_) {
    modes.push_back(joint.claimed);
  }
  return modes;
}

std::vector<bool> RobStrideDriver::feedback_received() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<bool> received;
  received.reserve(joints_.size());
  for (const auto & joint : joints_) {
    received.push_back(joint.feedback_status.received);
  }
  return received;
}

bool RobStrideDriver::apply_command_modes(const std::vector<ClaimedInterfaces> & modes)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (modes.size() != joints_.size()) {return false;}
  for (size_t index = 0; index < joints_.size(); ++index) {
    auto & joint = joints_[index];
    const auto previous = joint.claimed;
    joint.claimed = modes[index];
    if (!previous.position && joint.claimed.position) {joint.command.position = joint.state.position;}
    if (!previous.velocity && joint.claimed.velocity) {joint.command.velocity = 0.0;}
    if (!previous.effort && joint.claimed.effort) {joint.command.effort = 0.0;}
  }
  return true;
}

DriverMetrics RobStrideDriver::metrics() const
{
  DriverMetrics snapshot;
  const auto now = std::chrono::steady_clock::now();
  const int64_t now_ns = steady_time_ns(now);
  const int64_t started_at_ns = metrics_started_at_ns_.load();
  snapshot.observation_period = std::chrono::nanoseconds(
    started_at_ns > 0 ? std::max<int64_t>(0, now_ns - started_at_ns) : 0);
  snapshot.feedback_frames_received = feedback_frames_received_.load();
  snapshot.parameter_frames_received = parameter_frames_received_.load();
  snapshot.hardware_active = active_.load();
  if (transport_) {
    snapshot.transport = transport_->metrics();
    snapshot.transport_health = transport_->health(settings_.transmit_failure_timeout);
  }

  snapshot.motors.reserve(joints_.size());
  for (size_t index = 0; index < joints_.size(); ++index) {
    MotorFeedbackMetrics motor;
    motor.joint_name = joints_[index].name;
    motor.can_id = joints_[index].can_id;
    const auto feedback = feedback_metrics_[index].load();
    motor.feedback_frames_received = feedback.count;
    const int64_t last_received_at_ns = feedback.last_received_at_ns;
    motor.feedback_received = last_received_at_ns > 0;
    if (motor.feedback_received) {
      motor.current_feedback_age =
        std::chrono::nanoseconds(std::max<int64_t>(0, now_ns - last_received_at_ns));
      motor.maximum_feedback_age = std::max(
        motor.current_feedback_age,
        std::chrono::nanoseconds(feedback.maximum_gap_ns));
    } else {
      motor.current_feedback_age = snapshot.observation_period;
      motor.maximum_feedback_age = snapshot.observation_period;
    }
    motor.feedback_rate_hz =
      rate_hz(motor.feedback_frames_received, snapshot.observation_period);
    motor.temperature = feedback.temperature;
    motor.mode = feedback.mode;
    motor.fault_flags = feedback.fault_flags;
    motor.feedback_stale =
      motor.feedback_received && motor.current_feedback_age > settings_.feedback_timeout;
    motor.recovery_active = recovery_metrics_[index].active.load();
    motor.recovery_attempts = recovery_metrics_[index].attempts.load();
    snapshot.motors.push_back(std::move(motor));
  }
  return snapshot;
}

void RobStrideDriver::receive_frame(can_msgs::msg::Frame::ConstSharedPtr msg)
{
  if (msg->is_error || msg->is_rtr) {return;}
  const uint16_t data_area = static_cast<uint16_t>((msg->id >> 8) & 0xffff);
  const uint8_t motor_id = static_cast<uint8_t>(data_area & 0xff);
  const auto joint_entry = can_id_to_joint_.find(motor_id);
  if (joint_entry == can_id_to_joint_.end()) {return;}

  const auto parameter = decode_parameter_response(
    msg->id, msg->data, msg->dlc, msg->is_extended, settings_.host_id);
  if (parameter) {
    ++parameter_frames_received_;
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto & joint = joints_[joint_entry->second];
    joint.parameter_status.received = true;
    joint.parameter_status.index = parameter->index;
    joint.parameter_status.value = parameter->value;
    joint.parameter_status.timestamp = std::chrono::steady_clock::now();
    feedback_condition_.notify_all();
    return;
  }

  const auto decoded = decode_feedback(
    msg->id, msg->data, msg->dlc, msg->is_extended, settings_.host_id,
    joints_[joint_entry->second].limits);
  if (!decoded) {return;}
  const auto received_at = std::chrono::steady_clock::now();
  ++feedback_frames_received_;
  record_feedback(joint_entry->second, received_at, *decoded);
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto & joint = joints_[joint_entry->second];
  joint.feedback.position =
    joint.direction * (decoded->position / joint.gear_ratio) + joint.position_offset;
  joint.feedback.velocity = joint.direction * (decoded->velocity / joint.gear_ratio);
  joint.feedback.effort = joint.motor_to_joint_effort(decoded->effort);
  joint.feedback.temperature = decoded->temperature;
  joint.feedback.fault = decoded->fault_flags;
  joint.feedback_status.mode = decoded->mode;
  joint.feedback_status.received = true;
  joint.feedback_status.timestamp = received_at;
  feedback_condition_.notify_all();
}

void RobStrideDriver::reset_metrics()
{
  feedback_frames_received_ = 0;
  parameter_frames_received_ = 0;
  metrics_started_at_ns_ = steady_time_ns(std::chrono::steady_clock::now());
  if (!feedback_metrics_) {return;}
  for (size_t index = 0; index < joints_.size(); ++index) {
    feedback_metrics_[index].store(MotorFeedbackSample{});
    recovery_metrics_[index].active = false;
    recovery_metrics_[index].attempts = 0;
  }
}

void RobStrideDriver::record_feedback(
  size_t joint_index, std::chrono::steady_clock::time_point now,
  const Feedback & feedback) noexcept
{
  auto sample = feedback_metrics_[joint_index].load();
  const int64_t now_ns = steady_time_ns(now);
  const int64_t previous_ns = sample.last_received_at_ns;
  ++sample.count;
  sample.last_received_at_ns = now_ns;
  const int64_t reference_ns =
    previous_ns > 0 ? previous_ns : metrics_started_at_ns_.load();
  const int64_t gap_ns = std::max<int64_t>(0, now_ns - reference_ns);
  sample.maximum_gap_ns = std::max(sample.maximum_gap_ns, gap_ns);
  sample.mode = feedback.mode;
  sample.fault_flags = feedback.fault_flags;
  sample.temperature = feedback.temperature;
  feedback_metrics_[joint_index].store(sample);
}

bool RobStrideDriver::write_and_confirm_parameter(
  JointData & joint, uint16_t index, uint32_t value)
{
  for (int attempt = 1; attempt <= settings_.startup_retries; ++attempt) {
    const auto requested_at = std::chrono::steady_clock::now();
    if (index == kIndexRunMode) {
      transport_->send_transaction(
        make_write_u8(joint.can_id, settings_.host_id, index, static_cast<uint8_t>(value)));
    } else {
      transport_->send_transaction(make_write_u32(joint.can_id, settings_.host_id, index, value));
    }
    transport_->send_transaction(make_read_parameter(joint.can_id, settings_.host_id, index));

    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, settings_.startup_confirmation_timeout,
      [&joint, index, value, requested_at]() {
        return joint.parameter_status.received &&
               joint.parameter_status.timestamp >= requested_at &&
               joint.parameter_status.index == index && joint.parameter_status.value == value;
      });
    if (confirmed) {return true;}
    RCLCPP_WARN(
      logger_, "Joint '%s': parameter 0x%04X confirmation attempt %d/%d failed",
      joint.name.c_str(), index, attempt, settings_.startup_retries);
  }
  RCLCPP_ERROR(
    logger_, "Joint '%s': parameter 0x%04X could not be confirmed",
    joint.name.c_str(), index);
  return false;
}

bool RobStrideDriver::enable_and_confirm_all()
{
  for (int attempt = 1; attempt <= settings_.startup_retries; ++attempt) {
    const auto requested_at = std::chrono::steady_clock::now();
    for (const auto & joint : joints_) {
      transport_->send_transaction(make_enable(joint.can_id, settings_.host_id));
      transport_->send_transaction(make_motion_command(
          joint.can_id, joint.limits, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, settings_.startup_confirmation_timeout,
      [this, requested_at]() {
        return std::all_of(joints_.begin(), joints_.end(), [requested_at](const JointData & joint) {
          return joint.feedback_status.received &&
                 joint.feedback_status.timestamp >= requested_at &&
                 joint.feedback_status.mode == kMotorModeRun;
        });
      });
    if (confirmed) {return true;}
    RCLCPP_WARN(
      logger_, "Motor enable confirmation attempt %d/%d failed",
      attempt, settings_.startup_retries);
  }
  RCLCPP_ERROR(logger_, "Not all motors entered Run mode");
  return false;
}

void RobStrideDriver::disable_all()
{
  if (!transport_) {
    active_ = false;
    return;
  }
  active_ = false;
  transport_->disable_active_commands();
  const auto stop_started = std::chrono::steady_clock::now();
  bool stop_frames_acknowledged = false;
  for (int attempt = 0; attempt < settings_.stop_repetitions; ++attempt) {
    for (const auto & joint : joints_) {
      transport_->send_transaction(
        make_motion_command(joint.can_id, joint.limits, 0.0, 0.0, 0.0, 0.0, 0.0));
      transport_->send_transaction(make_stop(joint.can_id, settings_.host_id));
    }
    stop_frames_acknowledged =
      transport_->wait_for_transaction_acknowledgements(std::chrono::milliseconds(50));
    if (attempt + 1 < settings_.stop_repetitions && settings_.stop_interval.count() > 0) {
      std::this_thread::sleep_for(settings_.stop_interval);
    }
  }
  if (!stop_frames_acknowledged) {
    RCLCPP_WARN(
      logger_,
      "Stop frames were not acknowledged by DDS; transport shutdown will still drain its local "
      "transaction queue and the motor watchdog remains the final fallback");
  }

  if (settings_.stop_confirmation_timeout.count() > 0) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool confirmed = feedback_condition_.wait_for(
      lock, settings_.stop_confirmation_timeout,
      [this, stop_started]() {
        return std::all_of(joints_.begin(), joints_.end(), [stop_started](const JointData & joint) {
          return joint.feedback_status.received &&
                 joint.feedback_status.timestamp >= stop_started &&
                 joint.feedback_status.mode == kMotorModeReset;
        });
      });
    if (!confirmed) {
      RCLCPP_ERROR(
        logger_, "Motor stop was not confirmed on CAN; motor watchdog must force reset after timeout");
    }
  }
}

void RobStrideDriver::log_runtime_events()
{
  for (const auto & event : runtime_events_) {
    const auto & joint = joints_[event.joint_index];
    if (event.kind == RuntimeEventKind::feedback_timeout) {
      RCLCPP_ERROR_THROTTLE(
        logger_, *log_clock_, 1000, "Feedback timeout for joint '%s'", joint.name.c_str());
    } else if (event.kind == RuntimeEventKind::recovery_started) {
      RCLCPP_WARN(
        logger_, "Joint '%s' left Run mode while hardware is active (mode=%u); attempting recovery",
        joint.name.c_str(), static_cast<unsigned int>(event.mode));
    } else if (event.kind == RuntimeEventKind::recovered) {
      RCLCPP_INFO(
        logger_, "Joint '%s' recovered to Run mode after %d enable attempt(s)",
        joint.name.c_str(), event.attempts);
    } else if (event.kind == RuntimeEventKind::recovery_failed) {
      RCLCPP_ERROR(
        logger_, "Joint '%s' did not return to Run mode within %lld ms after %d enable attempt(s)",
        joint.name.c_str(), static_cast<long long>(settings_.recovery_timeout.count()),
        event.attempts);
    }
  }
}

}  // namespace robstride_driver
