#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "robstride_driver/config.hpp"
#include "robstride_driver/metrics.hpp"

namespace robstride_driver
{

class RobStrideDriver
{
public:
  explicit RobStrideDriver(rclcpp::Logger logger);
  ~RobStrideDriver() noexcept;

  RobStrideDriver(const RobStrideDriver &) = delete;
  RobStrideDriver & operator=(const RobStrideDriver &) = delete;

  bool initialize(DriverConfiguration configuration);
  std::vector<JointData> & joints() noexcept;

  bool open();
  void close() noexcept;
  bool start();
  void stop() noexcept;
  bool update_state();
  bool send_commands();
  std::vector<ClaimedInterfaces> command_modes() const;
  std::vector<bool> feedback_received() const;
  bool apply_command_modes(const std::vector<ClaimedInterfaces> & modes);
  DriverMetrics metrics() const;

private:
  enum class RuntimeEventKind
  {
    feedback_timeout,
    recovery_started,
    recovered,
    recovery_failed,
  };

  struct RuntimeEvent
  {
    RuntimeEventKind kind;
    size_t joint_index;
    uint8_t mode{0};
    int attempts{0};
  };

  void receive_frame(can_msgs::msg::Frame::ConstSharedPtr msg);
  bool write_and_confirm_parameter(JointData & joint, uint16_t index, uint32_t value);
  bool enable_and_confirm_all();
  void disable_all();
  void log_runtime_events();
  bool check_transport_health();
  void reset_metrics();
  void record_feedback(
    size_t joint_index, std::chrono::steady_clock::time_point now,
    const Feedback & feedback) noexcept;

  struct AtomicRecoveryMetrics
  {
    std::atomic<bool> active{false};
    std::atomic<uint64_t> attempts{0};
  };

  rclcpp::Logger logger_;
  DriverSettings settings_{};
  std::vector<JointData> joints_;
  std::unordered_map<uint8_t, size_t> can_id_to_joint_;
  mutable std::mutex state_mutex_;
  std::condition_variable feedback_condition_;
  std::vector<RuntimeEvent> runtime_events_;
  std::vector<CanTransport::MotorFrame> command_snapshot_;
  std::vector<CanTransport::RecoveryUpdate> recovery_updates_;
  std::atomic<bool> active_{false};
  std::chrono::steady_clock::time_point activated_at_{};
  std::shared_ptr<rclcpp::Clock> log_clock_;
  std::unique_ptr<CanTransport> transport_;
  std::atomic<uint64_t> feedback_frames_received_{0};
  std::atomic<uint64_t> parameter_frames_received_{0};
  std::atomic<int64_t> metrics_started_at_ns_{0};
  std::unique_ptr<AtomicMotorFeedback[]> feedback_metrics_;
  std::unique_ptr<AtomicRecoveryMetrics[]> recovery_metrics_;
};

}  // namespace robstride_driver
