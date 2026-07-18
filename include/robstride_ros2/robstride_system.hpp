#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#ifdef ROBSTRIDE_ROS2_USE_HARDWARE_COMPONENT_PARAMS
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#endif
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "robstride_ros2/protocol.hpp"

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
    StateValues received{};
    double command_position{0.0};
    double command_velocity{0.0};
    double command_effort{0.0};
    bool position_active{false};
    bool velocity_active{false};
    bool effort_active{false};
    bool feedback_received{false};
    uint8_t feedback_mode{0};
    std::chrono::steady_clock::time_point last_feedback{};
    bool parameter_received{false};
    uint16_t last_parameter_index{0};
    uint32_t last_parameter_value{0};
    std::chrono::steady_clock::time_point last_parameter_response{};
  };

  hardware_interface::CallbackReturn initialize_from_info(
    const hardware_interface::HardwareInfo & info);
  bool parse_joint(const hardware_interface::ComponentInfo & info, Joint & joint);
  void receive_frame(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void publish(const Frame & frame);
  bool wait_for_transport();
  bool write_and_confirm_parameter(Joint & joint, uint16_t index, uint32_t value);
  bool enable_and_confirm_all();
  void disable_all();
  void stop_executor();
  static bool parse_bool(const std::string & value);

  std::vector<Joint> joints_;
  std::unordered_map<uint8_t, size_t> can_id_to_joint_;
  std::mutex mutex_;
  std::condition_variable feedback_condition_;
  uint8_t host_id_{0xfd};
  std::string tx_topic_{"to_can_bus"};
  std::string rx_topic_{"from_can_bus"};
  size_t qos_depth_{500};
  int feedback_timeout_ms_{3000};
  bool fail_on_feedback_timeout_{true};
  bool clear_faults_on_activate_{true};
  bool set_zero_on_activate_{false};
  int shutdown_stop_repetitions_{3};
  int shutdown_stop_interval_ms_{20};
  int shutdown_confirmation_timeout_ms_{300};
  int startup_connection_timeout_ms_{3000};
  int startup_confirmation_timeout_ms_{500};
  int startup_retries_{3};
  std::atomic<bool> active_{false};
  std::chrono::steady_clock::time_point activated_at_{};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic<bool> spinning_{false};
};

}  // namespace robstride_ros2
