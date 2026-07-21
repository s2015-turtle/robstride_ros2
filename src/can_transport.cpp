#include "robstride_ros2/can_transport.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace robstride_ros2
{

CanTransport::CanTransport(CanTransportOptions options, ReceiveCallback receive_callback)
: options_(std::move(options)), receive_callback_(std::move(receive_callback))
{
  if (options_.node_name.empty()) {throw std::invalid_argument("node_name must not be empty");}
  if (options_.motor_count == 0) {throw std::invalid_argument("motor_count must be positive");}
  if (options_.receive_qos_depth == 0) {
    throw std::invalid_argument("receive_qos_depth must be positive");
  }
  if (options_.motion_frame_lifespan.count() <= 0) {
    throw std::invalid_argument("motion_frame_lifespan must be positive");
  }
  if (!receive_callback_) {throw std::invalid_argument("receive_callback must be set");}
  pending_motion_frames_.resize(options_.motor_count);
  pending_recovery_frames_.resize(options_.motor_count);
}

CanTransport::~CanTransport() noexcept
{
  try {stop();} catch (...) {}
}

void CanTransport::start()
{
  if (running_) {return;}

  node_ = std::make_shared<rclcpp::Node>(options_.node_name);
  const auto transaction_qos =
    rclcpp::QoS(rclcpp::KeepLast(std::max<size_t>(32, options_.motor_count * 4)))
    .reliable().durability_volatile();
  auto motion_qos =
    rclcpp::QoS(rclcpp::KeepLast(options_.motor_count))
    .reliable().durability_volatile();
  motion_qos.lifespan(rclcpp::Duration::from_seconds(
      static_cast<double>(options_.motion_frame_lifespan.count()) / 1000.0));
  const auto receive_qos =
    rclcpp::QoS(rclcpp::KeepLast(options_.receive_qos_depth))
    .reliable().durability_volatile();

  transaction_publisher_ =
    node_->create_publisher<can_msgs::msg::Frame>(options_.transmit_topic, transaction_qos);
  motion_publisher_ =
    node_->create_publisher<can_msgs::msg::Frame>(options_.transmit_topic, motion_qos);
  receive_subscription_ = node_->create_subscription<can_msgs::msg::Frame>(
    options_.receive_topic, receive_qos, receive_callback_);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  running_ = true;
  executor_thread_ = std::thread([this]() {executor_->spin();});
  worker_thread_ = std::thread([this]() {transmit_pending_frames();});
}

void CanTransport::stop()
{
  disable_active_commands();
  running_ = false;
  pending_condition_.notify_all();
  if (worker_thread_.joinable()) {worker_thread_.join();}
  discard_pending_frames();

  if (executor_) {executor_->cancel();}
  if (executor_thread_.joinable()) {executor_thread_.join();}

  receive_subscription_.reset();
  {
    std::lock_guard<std::mutex> lock(publisher_mutex_);
    motion_publisher_.reset();
    transaction_publisher_.reset();
  }
  if (executor_ && node_) {executor_->remove_node(node_);}
  executor_.reset();
  node_.reset();
}

bool CanTransport::wait_for_endpoints(std::chrono::milliseconds timeout) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (transaction_publisher_ && motion_publisher_ && receive_subscription_ &&
      transaction_publisher_->get_subscription_count() > 0 &&
      motion_publisher_->get_subscription_count() > 0 &&
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
  publish_transaction(frame);
}

void CanTransport::queue_motion_frame(size_t motor_index, const Frame & frame)
{
  if (!active_commands_enabled_ || motor_index >= options_.motor_count) {return;}
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!active_commands_enabled_) {return;}
    pending_motion_frames_[motor_index] = frame;
  }
  pending_condition_.notify_one();
}

void CanTransport::queue_recovery_frame(size_t motor_index, const Frame & frame)
{
  if (!active_commands_enabled_ || motor_index >= options_.motor_count) {return;}
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!active_commands_enabled_) {return;}
    pending_recovery_frames_[motor_index] = frame;
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
  discard_pending_frames();
  // Wait for a frame that already passed the enabled check. Stop transactions sent after this
  // method therefore always follow every in-flight recovery or motion publication.
  std::lock_guard<std::mutex> lock(publisher_mutex_);
}

bool CanTransport::wait_for_transaction_acknowledgements(std::chrono::milliseconds timeout) const
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  if (!transaction_publisher_ || transaction_publisher_->get_subscription_count() == 0) {
    return false;
  }
  return transaction_publisher_->wait_for_all_acked(timeout);
}

void CanTransport::publish_transaction(const Frame & frame)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  publish_unlocked(transaction_publisher_, frame);
}

void CanTransport::publish_motion(const Frame & frame)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  if (!active_commands_enabled_) {return;}
  publish_unlocked(motion_publisher_, frame);
}

void CanTransport::publish_recovery(const Frame & frame)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  if (!active_commands_enabled_) {return;}
  publish_unlocked(transaction_publisher_, frame);
}

void CanTransport::publish_unlocked(
  const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr & publisher,
  const Frame & source)
{
  if (!publisher) {return;}

  can_msgs::msg::Frame message;
  message.header.stamp = node_->now();
  message.id = source.id;
  message.is_rtr = false;
  message.is_extended = true;
  message.is_error = false;
  message.dlc = 8;
  message.data = source.data;
  publisher->publish(message);
}

void CanTransport::transmit_pending_frames()
{
  std::vector<Frame> recovery_frames;
  std::vector<Frame> motion_frames;
  recovery_frames.reserve(options_.motor_count);
  motion_frames.reserve(options_.motor_count);

  while (running_) {
    std::unique_lock<std::mutex> lock(pending_mutex_);
    pending_condition_.wait(lock, [this]() {
      if (!running_) {return true;}
      const bool has_motion = std::any_of(
        pending_motion_frames_.begin(), pending_motion_frames_.end(),
        [](const std::optional<Frame> & frame) {return frame.has_value();});
      const bool has_recovery = std::any_of(
        pending_recovery_frames_.begin(), pending_recovery_frames_.end(),
        [](const std::optional<Frame> & frame) {return frame.has_value();});
      return has_motion || has_recovery;
    });
    if (!running_) {break;}

    recovery_frames.clear();
    motion_frames.clear();
    for (auto & pending : pending_recovery_frames_) {
      if (!pending) {continue;}
      recovery_frames.push_back(*pending);
      pending.reset();
    }
    for (auto & pending : pending_motion_frames_) {
      if (!pending) {continue;}
      motion_frames.push_back(*pending);
      pending.reset();
    }
    lock.unlock();

    // Enable must precede a motion frame when both are pending for a reset motor.
    for (const auto & frame : recovery_frames) {publish_recovery(frame);}
    for (const auto & frame : motion_frames) {publish_motion(frame);}
  }
}

void CanTransport::discard_pending_frames()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  for (auto & pending : pending_motion_frames_) {pending.reset();}
  for (auto & pending : pending_recovery_frames_) {pending.reset();}
}

}  // namespace robstride_ros2
