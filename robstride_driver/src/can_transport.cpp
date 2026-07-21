#include "robstride_driver/can_transport.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace robstride_driver
{

CanTransport::CanTransport(
  CanTransportOptions options, ReceiveCallback receive_callback, FrameSink frame_sink)
: options_(std::move(options)),
  receive_callback_(std::move(receive_callback)),
  frame_sink_(std::move(frame_sink)),
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
  if (running_) {return;}

  if (!frame_sink_) {
    node_ = std::make_shared<rclcpp::Node>(options_.node_name);
    const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(std::max<size_t>(32, options_.motor_count * 4)))
      .reliable().durability_volatile();
    const auto receive_qos =
      rclcpp::QoS(rclcpp::KeepLast(options_.receive_qos_depth))
      .reliable().durability_volatile();

    publisher_ = node_->create_publisher<can_msgs::msg::Frame>(options_.transmit_topic, qos);
    receive_subscription_ = node_->create_subscription<can_msgs::msg::Frame>(
      options_.receive_topic, receive_qos, receive_callback_);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  running_ = true;
  if (executor_) {executor_thread_ = std::thread([this]() {executor_->spin();});}
  worker_thread_ = std::thread([this]() {transmit_pending_frames();});
}

void CanTransport::stop()
{
  disable_active_commands();
  running_ = false;
  pending_condition_.notify_all();
  if (worker_thread_.joinable()) {worker_thread_.join();}

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_transactions_.clear();
    transactions_in_flight_ = 0;
    for (auto & pending : pending_motion_frames_) {pending.reset();}
    for (auto & pending : pending_recovery_frames_) {pending.reset();}
  }

  if (executor_) {executor_->cancel();}
  if (executor_thread_.joinable()) {executor_thread_.join();}

  receive_subscription_.reset();
  {
    std::lock_guard<std::mutex> lock(publisher_mutex_);
    publisher_.reset();
  }
  if (executor_ && node_) {executor_->remove_node(node_);}
  executor_.reset();
  node_.reset();
}

bool CanTransport::wait_for_endpoints(std::chrono::milliseconds timeout) const
{
  if (frame_sink_) {return running_;}
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (publisher_ && receive_subscription_ && publisher_->get_subscription_count() > 0 &&
      receive_subscription_->get_publisher_count() > 0)
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
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
  if (!active_commands_enabled_ || motor_index >= options_.motor_count) {return;}
  const uint64_t generation = active_generation_;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!active_commands_enabled_ || generation != active_generation_) {return;}
    pending_motion_frames_[motor_index] = ActiveFrame{frame, motor_index, generation};
  }
  pending_condition_.notify_one();
}

void CanTransport::queue_recovery_frame(size_t motor_index, const Frame & frame)
{
  if (!active_commands_enabled_ || motor_index >= options_.motor_count) {return;}
  recovery_active_[motor_index] = true;
  const uint64_t generation = active_generation_;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!active_commands_enabled_ || generation != active_generation_) {
      recovery_active_[motor_index] = false;
      return;
    }
    pending_recovery_frames_[motor_index] = ActiveFrame{frame, motor_index, generation};
  }
  pending_condition_.notify_one();
}

void CanTransport::complete_recovery(size_t motor_index)
{
  if (motor_index >= options_.motor_count) {return;}
  recovery_active_[motor_index] = false;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_recovery_frames_[motor_index].reset();
  }
  pending_condition_.notify_one();
}

void CanTransport::enable_active_commands()
{
  disable_active_commands();
  active_commands_enabled_ = true;
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
}

bool CanTransport::wait_for_transaction_acknowledgements(
  std::chrono::milliseconds timeout) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  {
    std::unique_lock<std::mutex> lock(pending_mutex_);
    if (!pending_condition_.wait_until(lock, deadline, [this]() {
        return pending_transactions_.empty() && transactions_in_flight_ == 0;
      }))
    {
      return false;
    }
  }

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
  publish_unlocked(frame);
}

void CanTransport::publish_active(const ActiveFrame & frame, bool is_recovery)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  if (!active_commands_enabled_ || frame.generation != active_generation_) {return;}
  const bool recovering = recovery_active_[frame.motor_index];
  if ((is_recovery && !recovering) || (!is_recovery && recovering)) {return;}
  publish_unlocked(frame.frame);
}

void CanTransport::publish_unlocked(const Frame & source)
{
  if (frame_sink_) {
    frame_sink_(source);
    return;
  }
  if (!publisher_) {return;}

  can_msgs::msg::Frame message;
  message.header.stamp = node_->now();
  message.id = source.id;
  message.is_rtr = false;
  message.is_extended = true;
  message.is_error = false;
  message.dlc = 8;
  message.data = source.data;
  publisher_->publish(message);
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
    if (!running_ && pending_transactions_.empty()) {break;}

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

    for (const auto & frame : transactions) {publish_transaction(frame);}
    for (const auto & frame : recovery_frames) {publish_active(frame, true);}
    for (const auto & frame : motion_frames) {publish_active(frame, false);}

    if (!transactions.empty()) {
      std::lock_guard<std::mutex> completed_lock(pending_mutex_);
      transactions_in_flight_ -= transactions.size();
      pending_condition_.notify_all();
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
