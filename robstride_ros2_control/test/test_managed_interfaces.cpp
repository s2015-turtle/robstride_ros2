#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <thread>

#include <hardware_interface/handle.hpp>

#include "robstride_driver/driver.hpp"
#include "robstride_ros2_control/driver_config.hpp"
#include "robstride_ros2_control/interface_value_bindings.hpp"
#include "robstride_ros2_control/robstride_system.hpp"

namespace rs = robstride_ros2_control;
namespace hi = hardware_interface;

namespace
{
hi::InterfaceInfo interface(const std::string & name)
{
  hi::InterfaceInfo info{};
  info.name = name;
  info.data_type = "double";
  // URDF initial values must not override the driver's neutral/unknown targets.
  info.initial_value = "42.0";
  return info;
}

hi::HardwareComponentInterfaceParams parameters(bool optional_states = true)
{
  hi::HardwareComponentInterfaceParams params{};
  params.hardware_info.name = "test_managed_system";
  params.hardware_info.type = "system";
  for (int id = 1; id <= 2; ++id) {
    hi::ComponentInfo joint{};
    joint.name = "joint_" + std::to_string(id);
    joint.type = "joint";
    joint.parameters = {
      {"model", "EL05"}, {"can_id", std::to_string(id)}, {"can_timeout_ticks", "4000"},
      {"kp", "30.0"}, {"kd", "1.0"}};
    joint.command_interfaces = {interface("effort"), interface("position"), interface("velocity")};
    joint.state_interfaces = {interface("velocity"), interface("effort"), interface("position")};
    if (optional_states) {
      joint.state_interfaces.push_back(interface("fault"));
      joint.state_interfaces.push_back(interface("temperature"));
    }
    params.hardware_info.joints.push_back(joint);
  }
  return params;
}

template<typename Interface>
std::shared_ptr<Interface> handle(const std::string & joint, const std::string & name)
{
  return std::make_shared<Interface>(hi::InterfaceDescription(joint, interface(name)));
}

template<typename Interface>
double value(const std::shared_ptr<Interface> & handle)
{
  double result = 0.0;
  EXPECT_TRUE(handle->get_value(result, false));
  return result;
}

// Hold the handle from another thread to exercise try-lock failure deterministically.
class LockedHandle
{
public:
  explicit LockedHandle(std::shared_mutex & mutex)
  {
    auto ready = ready_.get_future();
    thread_ = std::thread([this, &mutex]() {
      std::unique_lock<std::shared_mutex> lock(mutex);
      ready_.set_value();
      release_.get_future().wait();
    });
    ready.wait();
  }
  ~LockedHandle()
  {
    release_.set_value();
    thread_.join();
  }

private:
  std::promise<void> ready_;
  std::promise<void> release_;
  std::thread thread_;
};
}  // namespace

TEST(ManagedInterfaces, ExportsDeclaredInterfacesAndDriverInitialValues)
{
  for (bool optional : {false, true}) {
    rs::RobStrideSystem system;
    ASSERT_EQ(system.on_init(parameters(optional)), hi::CallbackReturn::SUCCESS);
    const auto states = system.on_export_state_interfaces();
    const auto commands = system.on_export_command_interfaces();
    ASSERT_EQ(states.size(), optional ? 10u : 6u);
    ASSERT_EQ(commands.size(), 6u);
    std::set<std::string> state_names;
    for (const auto & state : states) {state_names.insert(state->get_name());}
    EXPECT_EQ(state_names.count("joint_1/temperature"), optional ? 1u : 0u);
    EXPECT_EQ(state_names.count("joint_2/fault"), optional ? 1u : 0u);
    EXPECT_TRUE(std::isnan(system.get_state("joint_1/position")));
    EXPECT_TRUE(std::isnan(system.get_command("joint_1/position")));
    EXPECT_DOUBLE_EQ(system.get_command("joint_2/velocity"), 0.0);
    EXPECT_DOUBLE_EQ(system.get_command("joint_1/effort"), 0.0);
    if (optional) {EXPECT_DOUBLE_EQ(system.get_state("joint_2/fault"), 0.0);}

    // read() must publish driver storage, not leave framework-local values untouched.
    system.set_state("joint_1/position", 9.0);
    EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hi::return_type::OK);
    EXPECT_TRUE(std::isnan(system.get_state("joint_1/position")));
  }
}

TEST(ManagedInterfaces, ModeSwitchResetsOnlyNewlyClaimedTargets)
{
  rs::RobStrideSystem system;
  ASSERT_EQ(system.on_init(parameters()), hi::CallbackReturn::SUCCESS);
  system.on_export_state_interfaces();
  system.on_export_command_interfaces();
  ASSERT_EQ(system.perform_command_mode_switch({"joint_2/effort"}, {}), hi::return_type::OK);
  system.set_command("joint_2/effort", 0.4);
  system.set_command("joint_1/velocity", 7.0);
  ASSERT_EQ(system.perform_command_mode_switch({"joint_1/velocity"}, {}), hi::return_type::OK);
  EXPECT_DOUBLE_EQ(system.get_command("joint_1/velocity"), 0.0);
  EXPECT_DOUBLE_EQ(system.get_command("joint_2/effort"), 0.4);
  system.set_command("joint_1/velocity", 1.5);
  ASSERT_EQ(system.perform_command_mode_switch({}, {}), hi::return_type::OK);
  EXPECT_DOUBLE_EQ(system.get_command("joint_1/velocity"), 1.5);
  ASSERT_EQ(system.perform_command_mode_switch(
      {"joint_1/velocity"}, {"joint_1/velocity"}), hi::return_type::OK);
  EXPECT_DOUBLE_EQ(system.get_command("joint_1/velocity"), 1.5);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hi::return_type::OK);
  // Position still requires fresh feedback; an old URDF initial value is not feedback.
  EXPECT_EQ(system.perform_command_mode_switch(
      {"joint_1/position"}, {"joint_1/velocity"}), hi::return_type::ERROR);
}

TEST(ManagedInterfaces, PositionInitializationSurvivesFirstCommandTransfer)
{
  robstride_driver::RobStrideDriver driver(rclcpp::get_logger("managed_interface_test"));
  ASSERT_TRUE(driver.initialize(rs::parse_driver_configuration(parameters().hardware_info)));
  auto & joint = driver.joints()[0];
  joint.feedback.position = -2.75;
  joint.feedback_status.received = true;
  joint.feedback_status.timestamp = std::chrono::steady_clock::now();
  const auto position = handle<hi::CommandInterface>(joint.name, "position");
  rs::InterfaceValueBindings<hi::CommandInterface> bindings;
  bindings.add(position, joint.command.position);

  auto modes = driver.command_modes();
  modes[0].position = true;
  ASSERT_TRUE(driver.apply_command_modes(modes));
  ASSERT_TRUE(bindings.publish_selected({"joint_1/position"}));
  ASSERT_TRUE(bindings.consume());
  EXPECT_DOUBLE_EQ(joint.command.position, -2.75);
  ASSERT_TRUE(position->set_value(1.25, false));
  ASSERT_TRUE(bindings.consume());
  EXPECT_DOUBLE_EQ(joint.command.position, 1.25);
}

TEST(ManagedInterfaces, TransfersStateAndCompleteCommandBatches)
{
  const auto state = handle<hi::StateInterface>("joint_1", "position");
  double position = -1.5;
  rs::InterfaceValueBindings<hi::StateInterface> states;
  states.add(state, position);
  ASSERT_TRUE(states.publish());
  EXPECT_DOUBLE_EQ(value(state), -1.5);
  position = 2.25;
  ASSERT_TRUE(states.publish());
  EXPECT_DOUBLE_EQ(value(state), 2.25);

  const auto first = handle<hi::CommandInterface>("joint_1", "velocity");
  const auto second = handle<hi::CommandInterface>("joint_2", "velocity");
  double first_command = 0.0;
  double second_command = 0.0;
  rs::InterfaceValueBindings<hi::CommandInterface> commands;
  commands.add(first, first_command);
  commands.add(second, second_command);
  ASSERT_TRUE(first->set_value(1.5, false));
  ASSERT_TRUE(second->set_value(-2.5, false));
  {
    LockedHandle locked(second->get_mutex());
    EXPECT_FALSE(commands.consume());
    EXPECT_DOUBLE_EQ(first_command, 0.0);
    EXPECT_DOUBLE_EQ(second_command, 0.0);
  }
  ASSERT_TRUE(commands.consume());
  EXPECT_DOUBLE_EQ(first_command, 1.5);
  EXPECT_DOUBLE_EQ(second_command, -2.5);
  {
    LockedHandle locked(state->get_mutex());
    EXPECT_FALSE(states.publish());
  }
}

TEST(ManagedInterfaces, AdapterReportsUnavailableCommandStorage)
{
  rs::RobStrideSystem system;
  ASSERT_EQ(system.on_init(parameters()), hi::CallbackReturn::SUCCESS);
  system.on_export_state_interfaces();
  system.on_export_command_interfaces();
  {
    LockedHandle locked(system.get_command_interface_handle("joint_1/velocity")->get_mutex());
    EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hi::return_type::ERROR);
  }
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hi::return_type::OK);
}

TEST(ManagedInterfaces, FailedModeInitializationCannotConsumeStaleTargets)
{
  rs::RobStrideSystem system;
  ASSERT_EQ(system.on_init(parameters()), hi::CallbackReturn::SUCCESS);
  system.on_export_state_interfaces();
  system.on_export_command_interfaces();
  system.set_command("joint_1/velocity", 20.0);
  {
    LockedHandle locked(system.get_command_interface_handle("joint_1/velocity")->get_mutex());
    EXPECT_EQ(system.perform_command_mode_switch({"joint_1/velocity"}, {}), hi::return_type::ERROR);
  }
  // Releasing the handle alone must not make the old nonzero value executable.
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hi::return_type::ERROR);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hi::return_type::ERROR);
  EXPECT_EQ(system.perform_command_mode_switch({}, {}), hi::return_type::ERROR);
}
