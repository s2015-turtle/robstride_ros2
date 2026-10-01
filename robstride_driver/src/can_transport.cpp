#include "robstride_driver/can_transport.hpp"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace robstride_driver
{
namespace
{
int64_t steady_now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

CanTransport::CanTransport(
  CanTransportOptions options, ReceiveCallback receive_callback, FrameSink frame_sink,
  MetricsProvider metrics_provider, EndpointProbe endpoint_probe)
: options_(std::move(options)),
  receive_callback_(std::move(receive_callback)),
  frame_sink_(std::move(frame_sink)),
  metrics_provider_(std::move(metrics_provider)),
  endpoint_probe_(std::move(endpoint_probe)),
  recovery_active_(std::make_unique<std::atomic<bool>[]>(options_.motor_count))
{
  if (options_.node_name.empty()) {throw std::invalid_argument("node_name must not be empty");}
  if (options_.motor_count == 0) {throw std::invalid_argument("motor_count must be positive");}
  if (options_.receive_qos_depth == 0) {
    throw std::invalid_argument("receive_qos_depth must be positive");
  }
  if (!receive_callback_) {throw std::invalid_argument("receive_callback must be set");}
  pending_motion_frames_.resize(options_.motor_count);
  pending_recovery_frames_.resize(options_.motor_count);
  for (size_t i = 0; i < options_.motor_count; ++i) {recovery_active_[i] = false;}
}

CanTransport::~CanTransport() noexcept
{
  try {stop();} catch (...) {}
}

void CanTransport::start()
{
  if (running_ && !worker_failed_ && !executor_failed_ && !context_shutdown_) {return;}
  // A failed run still owns joinable threads and ROS entities. Reap it before
  // assigning new threads (std::thread assignment would otherwise terminate).
  stop();
  reset_metrics();
  worker_failed_ = false;
  executor_failed_ = false;
  context_shutdown_ = false;
  executor_stop_requested_ = false;
  bridge_available_ = false;
  bridge_unavailable_since_ns_ = 0;
  active_work_progress_ns_ = 0;
  update_endpoint_status(frame_sink_ && !endpoint_probe_);

  try {
    if (!frame_sink_) {
      rclcpp::NodeOptions node_options;
      if (options_.context) {node_options.context(options_.context);}
      node_ = std::make_shared<rclcpp::Node>(options_.node_name, node_options);
      const auto qos =
        rclcpp::QoS(rclcpp::KeepLast(std::max<size_t>(32, options_.motor_count * 4)))
        .reliable().durability_volatile();
      const auto receive_qos =
        rclcpp::QoS(rclcpp::KeepLast(options_.receive_qos_depth))
        .reliable().durability_volatile();

      publisher_ = node_->create_publisher<can_msgs::msg::Frame>(options_.transmit_topic, qos);
      receive_subscription_ = node_->create_subscription<can_msgs::msg::Frame>(
        options_.receive_topic, receive_qos,
        [this](can_msgs::msg::Frame::ConstSharedPtr message) {
          try {receive_callback_(std::move(message));} catch (...) {record_failure(true);}
        });
      diagnostics_publisher_ =
        node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
      if (metrics_provider_) {
        diagnostics_timer_ = node_->create_wall_timer(
          std::chrono::seconds(1), [this]() {
            try {publish_diagnostics();} catch (...) {record_failure(true);}
          });
      }
      rclcpp::ExecutorOptions executor_options;
      executor_options.context = node_->get_node_base_interface()->get_context();
      executor_ =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
      executor_->add_node(node_);
    }

    running_ = true;
    if (executor_) {executor_thread_ = std::thread([this]() {run_executor();});}
    worker_thread_ = std::thread([this]() noexcept {
      // Includes allocation and wait/extraction, not just publisher calls.
      try {transmit_pending_frames();} catch (...) {record_failure(false);}
    });
  } catch (...) {
    record_failure(true);
    stop();
    throw;
  }
}

void CanTransport::stop()
{
  // Close the producer side before waiting for an in-flight publication.  Leaving the
  // transport running while disable_active_commands() waits on publisher_mutex_ lets the
  // worker extract more work and makes shutdown scheduling-dependent.  The worker still
  // drains transactions that were queued before this point, but no longer extracts active
  // command frames once running_ is false.
  {
    // Update the wait predicate under its mutex so the shutdown notification cannot
    // arrive between the worker's predicate check and its entry into the wait.
    std::lock_guard<std::mutex> lock(pending_mutex_);
    running_ = false;
  }
  pending_condition_.notify_all();
  executor_stop_requested_ = true;
  disable_active_commands();
  if (worker_thread_.joinable()) {worker_thread_.join();}

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_transactions_.clear();
    transactions_in_flight_ = 0;
    for (auto & pending : pending_motion_frames_) {pending.reset();}
    for (auto & pending : pending_recovery_frames_) {pending.reset();}
  }

  // spin_once has a finite wait, so joining does not depend on cancel()
  // successfully waking a ROS guard condition (which can throw on shutdown).
  if (executor_thread_.joinable()) {executor_thread_.join();}

  receive_subscription_.reset();
  diagnostics_timer_.reset();
  diagnostics_publisher_.reset();
  {
    std::lock_guard<std::mutex> lock(publisher_mutex_);
    publisher_.reset();
  }
  if (executor_ && node_) {
    try {executor_->remove_node(node_);} catch (...) {
      // Both threads are joined. Destruction of the executor releases the
      // association even when ROS teardown cannot notify its guard condition.
    }
  }
  executor_.reset();
  node_.reset();
}

bool CanTransport::wait_for_endpoints(std::chrono::milliseconds timeout) const
{
  if (frame_sink_) {
    bool available = running_;
    try {
      if (endpoint_probe_) {available = available && endpoint_probe_();}
    } catch (...) {
      available = false;
    }
    update_endpoint_status(available);
    return available;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (running_ && std::chrono::steady_clock::now() < deadline) {
    if (publisher_ && receive_subscription_ && publisher_->get_subscription_count() > 0 &&
      receive_subscription_->get_publisher_count() > 0)
    {
      update_endpoint_status(true);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  update_endpoint_status(false);
  return false;
}

void CanTransport::send_transaction(const Frame & frame)
{
  if (!running_) {return;}
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!running_) {return;}
    pending_transactions_.push_back(frame);
  }
  pending_condition_.notify_one();
}

void CanTransport::queue_motion_frame(size_t motor_index, const Frame & frame)
{
  queue_motion_frames({MotorFrame{motor_index, frame}});
}

void CanTransport::queue_motion_frames(const std::vector<MotorFrame> & frames)
{
  if (!active_commands_enabled_ || frames.empty()) {return;}
  const uint64_t generation = active_generation_;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!active_commands_enabled_ || generation != active_generation_) {return;}
    bool enqueued = false;
    for (const auto & update : frames) {
      if (update.motor_index >= options_.motor_count) {continue;}
      if (pending_motion_frames_[update.motor_index]) {++motion_frames_coalesced_;}
      pending_motion_frames_[update.motor_index] =
        ActiveFrame{update.frame, update.motor_index, generation};
      enqueued = true;
    }
    if (enqueued) {
      int64_t idle = 0;
      (void)active_work_progress_ns_.compare_exchange_strong(idle, steady_now_ns());
    }
  }
  pending_condition_.notify_one();
}

void CanTransport::queue_recovery_frame(size_t motor_index, const Frame & frame)
{
  apply_recovery_updates({RecoveryUpdate{motor_index, frame}});
}

void CanTransport::complete_recovery(size_t motor_index)
{
  apply_recovery_updates({RecoveryUpdate{motor_index, std::nullopt}});
}

void CanTransport::apply_recovery_updates(const std::vector<RecoveryUpdate> & updates)
{
  if (updates.empty()) {return;}
  const uint64_t generation = active_generation_;
  for (const auto & update : updates) {
    if (update.motor_index < options_.motor_count) {
      recovery_active_[update.motor_index] = update.frame.has_value();
    }
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!active_commands_enabled_ || generation != active_generation_) {
      for (const auto & update : updates) {
        if (update.motor_index < options_.motor_count) {
          recovery_active_[update.motor_index] = false;
        }
      }
      return;
    }
    bool enqueued = false;
    for (const auto & update : updates) {
      if (update.motor_index >= options_.motor_count) {continue;}
      if (update.frame) {
        pending_recovery_frames_[update.motor_index] =
          ActiveFrame{*update.frame, update.motor_index, generation};
        enqueued = true;
      } else {
        pending_recovery_frames_[update.motor_index].reset();
      }
    }
    if (enqueued) {
      int64_t idle = 0;
      (void)active_work_progress_ns_.compare_exchange_strong(idle, steady_now_ns());
    }
  }
  pending_condition_.notify_one();
}

void CanTransport::enable_active_commands()
{
  disable_active_commands();
  std::lock_guard<std::mutex> lock(pending_mutex_);
  active_commands_enabled_ =
    running_ && !worker_failed_ && !executor_failed_ && !context_shutdown_;
}

void CanTransport::disable_active_commands()
{
  active_commands_enabled_ = false;
  ++active_generation_;
  for (size_t i = 0; i < options_.motor_count; ++i) {recovery_active_[i] = false;}
  discard_pending_active_frames();
  // A frame already being published completes before lifecycle stop transactions are queued.
  // Extracted frames from an older generation are rejected even after a later reactivation.
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  active_work_progress_ns_ = 0;
}

bool CanTransport::wait_for_transaction_acknowledgements(
  std::chrono::milliseconds timeout) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  {
    std::unique_lock<std::mutex> lock(pending_mutex_);
    if (!pending_condition_.wait_until(lock, deadline, [this]() {
        return worker_failed_ || executor_failed_ || context_shutdown_ ||
               (pending_transactions_.empty() && transactions_in_flight_ == 0);
      }))
    {
      return false;
    }
  }

  if (worker_failed_ || executor_failed_ || context_shutdown_) {return false;}
  if (frame_sink_) {return true;}
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {return false;}
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  if (!publisher_ || publisher_->get_subscription_count() == 0) {return false;}
  return publisher_->wait_for_all_acked(
    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now));
}

void CanTransport::publish_transaction(const Frame & frame)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  if (worker_failed_ || executor_failed_ || context_shutdown_) {return;}
  if (publish_unlocked(frame)) {++transaction_frames_transmitted_;}
}

void CanTransport::publish_active(const ActiveFrame & frame, bool is_recovery)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  if (!running_ || worker_failed_ || executor_failed_ || context_shutdown_ ||
    !active_commands_enabled_ || frame.generation != active_generation_) {return;}
  const bool recovering = recovery_active_[frame.motor_index];
  if ((is_recovery && !recovering) || (!is_recovery && recovering)) {return;}
  if (!publish_unlocked(frame.frame)) {return;}
  if (is_recovery) {
    ++recovery_frames_transmitted_;
  } else {
    ++motion_frames_transmitted_;
  }
}

bool CanTransport::publish_unlocked(const Frame & source)
{
  bool endpoint_available = true;
  if (endpoint_probe_) {
    endpoint_available = endpoint_probe_();
  } else if (!frame_sink_) {
    endpoint_available = publisher_ && publisher_->get_subscription_count() > 0;
  }
  update_endpoint_status(endpoint_available);
  if (!endpoint_available) {return false;}

  if (frame_sink_) {
    frame_sink_(source);
    return true;
  }
  if (!publisher_) {return false;}

  can_msgs::msg::Frame message;
  message.header.stamp = node_->now();
  message.id = source.id;
  message.is_rtr = false;
  message.is_extended = true;
  message.is_error = false;
  message.dlc = 8;
  message.data = source.data;
  publisher_->publish(message);
  return true;
}

CanTransportMetrics CanTransport::metrics() const noexcept
{
  CanTransportMetrics snapshot;
  snapshot.motion_frames_transmitted = motion_frames_transmitted_.load();
  snapshot.recovery_frames_transmitted = recovery_frames_transmitted_.load();
  snapshot.transaction_frames_transmitted = transaction_frames_transmitted_.load();
  snapshot.motion_frames_coalesced = motion_frames_coalesced_.load();
  const int64_t started_at = metrics_started_at_ns_.load();
  snapshot.observation_period = std::chrono::nanoseconds(
    started_at > 0 ? std::max<int64_t>(0, steady_now_ns() - started_at) : 0);
  return snapshot;
}

CanTransportHealth CanTransport::health(
  std::chrono::milliseconds failure_timeout) const noexcept
{
  const int64_t now_ns = steady_now_ns();
  const auto elapsed_since = [now_ns](int64_t reference_ns) {
      return std::chrono::nanoseconds(
        reference_ns > 0 ? std::max<int64_t>(0, now_ns - reference_ns) : 0);
    };

  if (executor_failed_) {
    return CanTransportHealth{
      CanTransportHealthState::executor_failed, std::chrono::nanoseconds(0), true};
  }
  if (context_shutdown_) {
    return CanTransportHealth{
      CanTransportHealthState::context_shutdown, std::chrono::nanoseconds(0), true};
  }
  if (worker_failed_ || !running_) {
    return CanTransportHealth{
      CanTransportHealthState::worker_stopped, std::chrono::nanoseconds(0), true};
  }
  if (!bridge_available_) {
    const auto duration = elapsed_since(bridge_unavailable_since_ns_);
    return CanTransportHealth{
      CanTransportHealthState::bridge_unavailable, duration,
      duration >= failure_timeout};
  }
  if (active_commands_enabled_) {
    const int64_t progress_ns = active_work_progress_ns_.load();
    const auto duration = elapsed_since(progress_ns);
    if (progress_ns > 0 && duration >= failure_timeout) {
      return CanTransportHealth{
        CanTransportHealthState::transmit_stalled, duration, true};
    }
  }
  return CanTransportHealth{};
}

void CanTransport::reset_metrics() noexcept
{
  motion_frames_transmitted_ = 0;
  recovery_frames_transmitted_ = 0;
  transaction_frames_transmitted_ = 0;
  motion_frames_coalesced_ = 0;
  metrics_started_at_ns_ = steady_now_ns();
}

void CanTransport::update_endpoint_status(bool available) const noexcept
{
  bridge_available_ = available;
  if (available) {
    bridge_unavailable_since_ns_ = 0;
    return;
  }
  int64_t unset = 0;
  (void)bridge_unavailable_since_ns_.compare_exchange_strong(unset, steady_now_ns());
}

void CanTransport::record_failure(bool executor_failure) noexcept
{
  if (executor_failure) {executor_failed_ = true;} else {worker_failed_ = true;}
  // Match the worker's wait predicate and enable transition lock.
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    active_commands_enabled_ = false;
    ++active_generation_;
    running_ = false;
  }
  executor_stop_requested_ = true;
  pending_condition_.notify_all();
  // No provider call, ROS logging dependency, or exception may escape this path.
  std::fputs(executor_failure ? "RobStride CAN executor failed\n" :
    "RobStride CAN transmit worker failed\n", stderr);
}

void CanTransport::run_executor() noexcept
{
  try {
    const auto context = node_->get_node_base_interface()->get_context();
    while (!executor_stop_requested_ && rclcpp::ok(context)) {
      executor_->spin_once(std::chrono::milliseconds(50));
    }
    if (!executor_stop_requested_) {
      context_shutdown_ = true;
      // Context shutdown is expected, not an executor fault. Still stop TX and
      // reject active work so an active hardware interface observes lost service.
      std::lock_guard<std::mutex> lock(pending_mutex_);
      active_commands_enabled_ = false;
      running_ = false;
      pending_condition_.notify_all();
    }
  } catch (...) {
    // ROS wait sets may throw during normal context shutdown. Callback wrappers
    // above latch genuine user callback failures even if shutdown races them.
    if (rclcpp::ok(node_->get_node_base_interface()->get_context())) {
      record_failure(true);
    } else {
      context_shutdown_ = true;
      std::lock_guard<std::mutex> lock(pending_mutex_);
      active_commands_enabled_ = false;
      running_ = false;
      pending_condition_.notify_all();
    }
  }
  if (executor_failed_ || worker_failed_) {publish_failure_diagnostic();}
}

void CanTransport::publish_failure_diagnostic() noexcept
{
  try {
    if (!diagnostics_publisher_) {return;}
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = node_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = "robstride_driver/CAN traffic";
    status.hardware_id = options_.transmit_topic + " -> " + options_.receive_topic;
    status.message = executor_failed_ ? "executor failed" : "worker stopped";
    message.status.push_back(std::move(status));
    diagnostics_publisher_->publish(message);
  } catch (...) {
    // ROS or allocation itself may be unavailable. The latched health state and
    // stderr report remain usable without the failed diagnostic provider.
  }
}

void CanTransport::publish_diagnostics()
{
  if (!diagnostics_publisher_ || !metrics_provider_) {return;}
  const auto snapshot = metrics_provider_();
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = node_->now();

  auto value = [](const std::string & key, const auto & data) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      std::ostringstream stream;
      stream << data;
      item.value = stream.str();
      return item;
    };
  auto decimal = [](const std::string & key, double data) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(2) << data;
      item.value = stream.str();
      return item;
    };
  auto hex_value = [](const std::string & key, uint64_t data) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      std::ostringstream stream;
      stream << "0x" << std::hex << std::uppercase << data;
      item.value = stream.str();
      return item;
    };

  diagnostic_msgs::msg::DiagnosticStatus transport_status;
  const auto & transport_health = snapshot.transport_health;
  transport_status.level = transport_health.state == CanTransportHealthState::healthy ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    (transport_health.persistent && snapshot.hardware_active ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    diagnostic_msgs::msg::DiagnosticStatus::WARN);
  transport_status.name = "robstride_driver/CAN traffic";
  transport_status.hardware_id = options_.transmit_topic + " -> " + options_.receive_topic;
  transport_status.message = transport_health_name(transport_health.state);
  transport_status.values = {
    value("health", transport_health_name(transport_health.state)),
    value("failure_persistent", transport_health.persistent),
    decimal(
      "health_duration_ms",
      std::chrono::duration<double, std::milli>(transport_health.duration).count()),
    value("tx_frames", snapshot.transport.transmitted_frames()),
    decimal("tx_rate_hz", snapshot.transport.transmit_rate_hz()),
    value("tx_motion_frames", snapshot.transport.motion_frames_transmitted),
    value("tx_recovery_frames", snapshot.transport.recovery_frames_transmitted),
    value("tx_transaction_frames", snapshot.transport.transaction_frames_transmitted),
    value("coalesced_motion_frames", snapshot.transport.motion_frames_coalesced),
    value("rx_robstride_frames", snapshot.received_frames()),
    decimal("rx_robstride_rate_hz", snapshot.receive_rate_hz())};
  message.status.push_back(std::move(transport_status));

  for (const auto & motor : snapshot.motors) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    const bool has_fault = motor.fault_flags != 0;
    const bool unexpected_mode = snapshot.hardware_active && motor.feedback_received &&
      motor.mode != kMotorModeRun;
    if (has_fault || motor.feedback_stale) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (!motor.feedback_received || unexpected_mode || motor.recovery_active) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }
    status.name = "robstride_driver/" + motor.joint_name;
    status.hardware_id = "CAN ID " + std::to_string(motor.can_id);
    if (has_fault) {
      status.message = "Motor fault: " + motor_fault_summary(motor.fault_flags);
    } else if (motor.feedback_stale) {
      status.message = "Motor feedback is stale";
    } else if (!motor.feedback_received) {
      status.message = "No feedback received";
    } else if (motor.recovery_active) {
      status.message = "Recovering motor to Run mode";
    } else if (unexpected_mode) {
      status.message = std::string("Unexpected motor mode: ") + motor_mode_name(motor.mode);
    } else {
      status.message = "Motor feedback is healthy";
    }
    status.values = {
      value("mode", motor_mode_name(motor.mode)),
      value("mode_raw", static_cast<unsigned int>(motor.mode)),
      decimal("temperature_c", motor.temperature),
      hex_value("fault_flags_raw", motor.fault_flags),
      value("faults", motor_fault_summary(motor.fault_flags)),
      value("feedback_stale", motor.feedback_stale),
      value("recovery_active", motor.recovery_active),
      value("recovery_attempts", motor.recovery_attempts),
      value("feedback_frames", motor.feedback_frames_received),
      decimal("feedback_rate_hz", motor.feedback_rate_hz),
      decimal(
        "current_feedback_age_ms",
        std::chrono::duration<double, std::milli>(motor.current_feedback_age).count()),
      decimal(
        "maximum_feedback_age_ms",
        std::chrono::duration<double, std::milli>(motor.maximum_feedback_age).count())};
    message.status.push_back(std::move(status));
  }
  diagnostics_publisher_->publish(message);
}

void CanTransport::transmit_pending_frames()
{
  std::deque<Frame> transactions;
  std::vector<ActiveFrame> recovery_frames;
  std::vector<ActiveFrame> motion_frames;
  recovery_frames.reserve(options_.motor_count);
  motion_frames.reserve(options_.motor_count);

  while (true) {
    std::unique_lock<std::mutex> lock(pending_mutex_);
    pending_condition_.wait(lock, [this]() {
      return !running_ || !pending_transactions_.empty() || has_sendable_active_frame();
    });
    if (worker_failed_ || executor_failed_ || context_shutdown_ ||
      (!running_ && pending_transactions_.empty())) {break;}

    transactions.clear();
    recovery_frames.clear();
    motion_frames.clear();
    transactions.swap(pending_transactions_);
    transactions_in_flight_ += transactions.size();
    if (running_) {
      for (auto & pending : pending_recovery_frames_) {
        if (!pending) {continue;}
        recovery_frames.push_back(*pending);
        pending.reset();
      }
      for (size_t i = 0; i < pending_motion_frames_.size(); ++i) {
        auto & pending = pending_motion_frames_[i];
        if (!pending || recovery_active_[i]) {continue;}
        motion_frames.push_back(*pending);
        pending.reset();
      }
    }
    lock.unlock();

    try {
      for (const auto & frame : transactions) {publish_transaction(frame);}
      for (const auto & frame : recovery_frames) {
        publish_active(frame, true);
        active_work_progress_ns_ = steady_now_ns();
      }
      for (const auto & frame : motion_frames) {
        publish_active(frame, false);
        active_work_progress_ns_ = steady_now_ns();
      }
    } catch (...) {
      record_failure(false);
      return;
    }

    if (!transactions.empty()) {
      std::lock_guard<std::mutex> completed_lock(pending_mutex_);
      transactions_in_flight_ -= transactions.size();
      pending_condition_.notify_all();
    }
    {
      std::lock_guard<std::mutex> completed_lock(pending_mutex_);
      if (!has_sendable_active_frame()) {active_work_progress_ns_ = 0;}
    }
  }
}

void CanTransport::discard_pending_active_frames()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  for (auto & pending : pending_motion_frames_) {pending.reset();}
  for (auto & pending : pending_recovery_frames_) {pending.reset();}
}

bool CanTransport::has_sendable_active_frame() const
{
  const bool has_recovery = std::any_of(
    pending_recovery_frames_.begin(), pending_recovery_frames_.end(),
    [](const std::optional<ActiveFrame> & frame) {return frame.has_value();});
  if (has_recovery) {return true;}
  for (size_t i = 0; i < pending_motion_frames_.size(); ++i) {
    if (pending_motion_frames_[i] && !recovery_active_[i]) {return true;}
  }
  return false;
}

}  // namespace robstride_driver
