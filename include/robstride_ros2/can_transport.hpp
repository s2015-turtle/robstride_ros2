#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robstride_ros2/protocol.hpp"

namespace robstride_ros2
{

struct CanTransportOptions
{
  std::string node_name;
  std::string transmit_topic{"to_can_bus"};
  std::string receive_topic{"from_can_bus"};
  size_t motor_count{0};
  size_t receive_qos_depth{32};
  std::chrono::milliseconds motion_frame_lifespan{50};
};

class CanTransport
{
public:
  using ReceiveCallback = std::function<void(can_msgs::msg::Frame::ConstSharedPtr)>;

  CanTransport(CanTransportOptions options, ReceiveCallback receive_callback);
  ~CanTransport() noexcept;

  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;

  void start();
  void stop();
  bool wait_for_endpoints(std::chrono::milliseconds timeout) const;

  void send_transaction(const Frame & frame);
  void queue_motion_frame(size_t motor_index, const Frame & frame);
  void queue_recovery_frame(size_t motor_index, const Frame & frame);
  void enable_active_commands();
  void disable_active_commands();
  bool wait_for_transaction_acknowledgements(std::chrono::milliseconds timeout) const;

private:
  void publish_transaction(const Frame & frame);
  void publish_motion(const Frame & frame);
  void publish_recovery(const Frame & frame);
  void publish_unlocked(
    const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr & publisher,
    const Frame & frame);
  void transmit_pending_frames();
  void discard_pending_frames();

  CanTransportOptions options_;
  ReceiveCallback receive_callback_;
  std::vector<std::optional<Frame>> pending_motion_frames_;
  std::vector<std::optional<Frame>> pending_recovery_frames_;

  mutable std::mutex publisher_mutex_;
  std::mutex pending_mutex_;
  std::condition_variable pending_condition_;
  std::thread worker_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> active_commands_enabled_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr transaction_publisher_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr motion_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr receive_subscription_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

}  // namespace robstride_ros2
