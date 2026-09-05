#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robstride_driver/protocol.hpp"
#include "robstride_driver/metrics.hpp"

namespace robstride_driver
{

struct CanTransportOptions
{
  std::string node_name;
  std::string transmit_topic{"to_can_bus"};
  std::string receive_topic{"from_can_bus"};
  size_t motor_count{0};
  size_t receive_qos_depth{32};
};

class CanTransport
{
public:
  using ReceiveCallback = std::function<void(can_msgs::msg::Frame::ConstSharedPtr)>;
  using FrameSink = std::function<void(const Frame &)>;
  using MetricsProvider = std::function<DriverMetrics()>;
  using EndpointProbe = std::function<bool()>;

  struct MotorFrame
  {
    size_t motor_index{0};
    Frame frame{};
  };

  struct RecoveryUpdate
  {
    size_t motor_index{0};
    std::optional<Frame> frame;
  };

  CanTransport(
      CanTransportOptions options, ReceiveCallback receive_callback,
      FrameSink frame_sink = FrameSink{}, MetricsProvider metrics_provider = MetricsProvider{},
      EndpointProbe endpoint_probe = EndpointProbe{});
  ~CanTransport() noexcept;

  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;

  void start();
  void stop();
  bool wait_for_endpoints(std::chrono::milliseconds timeout) const;

  void send_transaction(const Frame & frame);
  void queue_motion_frame(size_t motor_index, const Frame & frame);
  void queue_motion_frames(const std::vector<MotorFrame> & frames);
  void queue_recovery_frame(size_t motor_index, const Frame & frame);
  void complete_recovery(size_t motor_index);
  void apply_recovery_updates(const std::vector<RecoveryUpdate> & updates);
  void enable_active_commands();
  void disable_active_commands();
  bool wait_for_transaction_acknowledgements(std::chrono::milliseconds timeout) const;
  CanTransportMetrics metrics() const noexcept;
  CanTransportHealth health(std::chrono::milliseconds failure_timeout) const noexcept;

private:
  struct ActiveFrame
  {
    Frame frame;
    size_t motor_index{0};
    uint64_t generation{0};
  };

  void publish_transaction(const Frame & frame);
  void publish_active(const ActiveFrame & frame, bool is_recovery);
  bool publish_unlocked(const Frame & frame);
  void transmit_pending_frames();
  void discard_pending_active_frames();
  bool has_sendable_active_frame() const;
  void reset_metrics() noexcept;
  void publish_diagnostics();
  void update_endpoint_status(bool available) const noexcept;

  CanTransportOptions options_;
  ReceiveCallback receive_callback_;
  FrameSink frame_sink_;
  MetricsProvider metrics_provider_;
  EndpointProbe endpoint_probe_;
  std::deque<Frame> pending_transactions_;
  std::vector<std::optional<ActiveFrame>> pending_motion_frames_;
  std::vector<std::optional<ActiveFrame>> pending_recovery_frames_;
  std::unique_ptr<std::atomic<bool>[]> recovery_active_;

  mutable std::mutex publisher_mutex_;
  mutable std::mutex pending_mutex_;
  mutable std::condition_variable pending_condition_;
  std::thread worker_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> worker_failed_{false};
  std::atomic<bool> active_commands_enabled_{false};
  std::atomic<uint64_t> active_generation_{0};
  size_t transactions_in_flight_{0};

  std::atomic<uint64_t> motion_frames_transmitted_{0};
  std::atomic<uint64_t> recovery_frames_transmitted_{0};
  std::atomic<uint64_t> transaction_frames_transmitted_{0};
  std::atomic<uint64_t> motion_frames_coalesced_{0};
  std::atomic<int64_t> metrics_started_at_ns_{0};
  mutable std::atomic<bool> bridge_available_{false};
  mutable std::atomic<int64_t> bridge_unavailable_since_ns_{0};
  std::atomic<int64_t> active_work_progress_ns_{0};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr receive_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

}  // namespace robstride_driver
