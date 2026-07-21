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
#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#endif
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "robstride_ros2/can_transport.hpp"
#include "robstride_ros2/protocol.hpp"
#include "robstride_ros2/run_mode_recovery.hpp"

namespace robstride_ros2
{

class RobStrideSystem : public hardware_interface::SystemInterface
{
public:
  ~RobStrideSystem() override;

#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#endif
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  struct StateValues
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double temperature{0.0};
    double fault{0.0};
  };

  struct CommandValues
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  struct ClaimedInterfaces
  {
    bool position{false};
    bool velocity{false};
    bool effort{false};
  };

  struct FeedbackStatus
  {
    bool received{false};
    uint8_t mode{0};
    std::chrono::steady_clock::time_point timestamp{};
  };

  struct ParameterStatus
  {
    bool received{false};
    uint16_t index{0};
    uint32_t value{0};
    std::chrono::steady_clock::time_point timestamp{};
  };

  struct Joint
  {
    std::string name;
    uint8_t can_id{0};
    Limits limits{};
    double direction{1.0};
    double gear_ratio{1.0};
    double position_offset{0.0};
    double kp{0.0};
    double kd{0.0};
    uint32_t can_timeout_ticks{0};
    StateValues state{};
    StateValues feedback{};
    CommandValues command{};
    ClaimedInterfaces claimed{};
    FeedbackStatus feedback_status{};
    ParameterStatus parameter_status{};
    RunModeRecoveryState run_mode_recovery{};
  };

  struct DriverSettings
  {
    uint8_t host_id{0xfd};
    CanTransportOptions transport{};
    std::chrono::milliseconds feedback_timeout{3000};
    bool fail_on_feedback_timeout{true};
    std::chrono::milliseconds run_mode_recovery_timeout{500};
    std::chrono::milliseconds run_mode_recovery_retry_interval{100};
    bool clear_faults_on_activate{true};
    bool set_zero_on_activate{false};
    int shutdown_stop_repetitions{3};
    std::chrono::milliseconds shutdown_stop_interval{20};
    std::chrono::milliseconds shutdown_confirmation_timeout{300};
    std::chrono::milliseconds startup_connection_timeout{3000};
    std::chrono::milliseconds startup_confirmation_timeout{500};
    int startup_retries{3};
  };

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

  hardware_interface::CallbackReturn initialize_from_info(
    const hardware_interface::HardwareInfo & info);
  bool parse_joint(const hardware_interface::ComponentInfo & info, Joint & joint);
  void receive_frame(const can_msgs::msg::Frame::ConstSharedPtr msg);
  bool write_and_confirm_parameter(Joint & joint, uint16_t index, uint32_t value);
  bool enable_and_confirm_all();
  void disable_all();
  void stop_transport();
  static bool parse_bool(const std::string & value);

  std::vector<Joint> joints_;
  std::unordered_map<uint8_t, size_t> can_id_to_joint_;
  std::mutex state_mutex_;
  std::condition_variable feedback_condition_;
  std::vector<RuntimeEvent> runtime_events_;
  DriverSettings settings_{};
  std::atomic<bool> active_{false};
  std::chrono::steady_clock::time_point activated_at_{};
  std::shared_ptr<rclcpp::Clock> log_clock_{
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
  std::unique_ptr<CanTransport> transport_;
};

}  // namespace robstride_ros2
