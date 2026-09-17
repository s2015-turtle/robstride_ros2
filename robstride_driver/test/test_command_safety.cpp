#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/logging.hpp>

#include "robstride_driver/driver.hpp"

namespace rs = robstride_driver;
using namespace std::chrono_literals;

namespace
{
rs::DriverConfiguration configuration()
{
  rs::DriverConfiguration config;
  config.settings.transport.node_name = "command_safety_test";
  config.settings.feedback_timeout = 100ms;
  rs::JointData joint;
  joint.name = "joint";
  joint.can_id = 1;
  joint.can_timeout_ticks = 4000;
  joint.limits = {-12.0, 12.0, -50.0, 50.0, -6.0, 6.0, -6.0, 6.0, 500.0, 5.0};
  joint.command_limits = {-1.0, 1.0, -10.0, 10.0, -3.0, 3.0};
  joint.direction = -1.0;
  joint.gear_ratio = 2.0;
  joint.position_offset = 0.25;
  config.joints.push_back(joint);
  return config;
}

void set_feedback(rs::RobStrideDriver & driver, double position)
{
  auto & joint = driver.joints().front();
  joint.feedback.position = position;
  joint.feedback_status.received = true;
  joint.feedback_status.timestamp = std::chrono::steady_clock::now();
}
}  // namespace

TEST(CommandSafety, RejectsMissingStaleNonfiniteAndOutsideFeedback)
{
  rs::RobStrideDriver driver(rclcpp::get_logger("command_safety"));
  ASSERT_TRUE(driver.initialize(configuration()));
  const std::vector<rs::ClaimedInterfaces> position{{true, false, false}};
  std::string error;
  EXPECT_FALSE(driver.validate_command_modes(position, &error));
  EXPECT_NE(error.find("fresh feedback"), std::string::npos);
  set_feedback(driver, 2.0);
  EXPECT_FALSE(driver.validate_command_modes(position, &error));
  EXPECT_NE(error.find("command_position_min/max"), std::string::npos);
  EXPECT_FALSE(driver.apply_command_modes(position, &error));
  EXPECT_FALSE(driver.command_modes()[0].position);
  set_feedback(driver, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(driver.apply_command_modes(position, &error));
  set_feedback(driver, 0.0);
  driver.joints()[0].feedback_status.timestamp -= 200ms;
  EXPECT_FALSE(driver.apply_command_modes(position, &error));
  EXPECT_FALSE(driver.command_modes()[0].position);
}

TEST(CommandSafety, UsesLatestFeedbackAndPreservesModeOnRevalidationFailure)
{
  rs::RobStrideDriver driver(rclcpp::get_logger("command_safety"));
  ASSERT_TRUE(driver.initialize(configuration()));
  const std::vector<rs::ClaimedInterfaces> position{{true, false, false}};
  set_feedback(driver, 0.0);
  EXPECT_TRUE(driver.validate_command_modes(position));
  set_feedback(driver, 1.1);
  EXPECT_FALSE(driver.apply_command_modes(position));
  EXPECT_FALSE(driver.command_modes()[0].position);
  set_feedback(driver, -1.0);
  EXPECT_TRUE(driver.apply_command_modes(position));
  EXPECT_DOUBLE_EQ(driver.joints()[0].command.position, -1.0);
  EXPECT_TRUE(driver.command_modes()[0].position);
  EXPECT_TRUE(driver.apply_command_modes({rs::ClaimedInterfaces{false, false, false}}));
  set_feedback(driver, 1.0);
  EXPECT_TRUE(driver.apply_command_modes(position));
  EXPECT_DOUBLE_EQ(driver.joints()[0].command.position, 1.0);
}
