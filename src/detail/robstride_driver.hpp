#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "detail/driver_config.hpp"

namespace robstride_ros2::detail
{

class RobStrideDriver
{
public:
  explicit RobStrideDriver(rclcpp::Logger logger);
  ~RobStrideDriver() noexcept;

  RobStrideDriver(const RobStrideDriver &) = delete;
  RobStrideDriver & operator=(const RobStrideDriver &) = delete;

  bool initialize(const hardware_interface::HardwareInfo & info);
  std::vector<JointData> & joints() noexcept;

  bool open();
  void close() noexcept;
  bool start();
  void stop() noexcept;
  bool update_state();
  void send_commands();
  bool validate_command_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop);
  void apply_command_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop);

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

  rclcpp::Logger logger_;
  DriverSettings settings_{};
  std::vector<JointData> joints_;
  std::unordered_map<uint8_t, size_t> can_id_to_joint_;
  std::mutex state_mutex_;
  std::condition_variable feedback_condition_;
  std::vector<RuntimeEvent> runtime_events_;
  std::atomic<bool> active_{false};
  std::chrono::steady_clock::time_point activated_at_{};
  std::shared_ptr<rclcpp::Clock> log_clock_;
  std::unique_ptr<CanTransport> transport_;
};

}  // namespace robstride_ros2::detail
