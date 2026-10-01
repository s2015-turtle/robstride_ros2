#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "robstride_ros2_control/command_mode.hpp"

namespace rs = robstride_ros2_control;

TEST(CommandMode, AllowsSwitchAfterStoppingCurrentInterface)
{
  const std::vector<rs::CommandModeState> current{{"joint_1", true, false, false}};
  EXPECT_TRUE(rs::validate_command_mode_switch(
      current, {"joint_1/velocity"}, {"joint_1/position"}));
}

TEST(CommandMode, RejectsSimultaneousInterfacesForOneJoint)
{
  const std::vector<rs::CommandModeState> current{{"joint_1", false, false, false}};
  std::string error;
  EXPECT_FALSE(rs::validate_command_mode_switch(
      current, {"joint_1/position", "joint_1/velocity"}, {}, &error));
  EXPECT_NE(error.find("joint_1"), std::string::npos);

  const std::vector<rs::CommandModeState> velocity_active{{"joint_1", false, true, false}};
  EXPECT_FALSE(rs::validate_command_mode_switch(
      velocity_active, {"joint_1/effort"}, {}));
}

TEST(CommandMode, AllowsDifferentInterfacesOnDifferentJoints)
{
  const std::vector<rs::CommandModeState> current{
    {"joint_1", false, false, false}, {"joint_2", false, false, false}};
  EXPECT_TRUE(rs::validate_command_mode_switch(
      current, {"joint_1/velocity", "joint_2/position"}, {}));
}

TEST(CommandMode, RejectsUnsupportedOwnedInterface)
{
  const std::vector<rs::CommandModeState> current{{"joint_1", false, false, false}};
  EXPECT_FALSE(rs::validate_command_mode_switch(current, {"joint_1/temperature"}, {}));
  EXPECT_FALSE(rs::validate_command_mode_switch(current, {}, {"joint_1/temperature"}));
}

TEST(CommandMode, ProducesModesForDriverWithoutRosInterfaceNames)
{
  const std::vector<rs::CommandModeState> current{{"joint_1", true, false, false}};
  const auto result = rs::command_modes_after_switch(
    current, {"joint_1/velocity"}, {"joint_1/position"});
  ASSERT_EQ(result.size(), 1U);
  EXPECT_FALSE(result[0].position_active);
  EXPECT_TRUE(result[0].velocity_active);
  EXPECT_FALSE(result[0].effort_active);
}

TEST(CommandMode, IgnoresForeignStartAndStopInterfaces)
{
  const std::vector<rs::CommandModeState> current{{"joint_1", true, false, false}};
  EXPECT_TRUE(rs::validate_command_mode_switch(
      current, {"other/position", "other/velocity"}, {"other/effort"}));
  const auto result = rs::command_modes_after_switch(
    current, {"other/position", "other/velocity"}, {"other/effort"});
  ASSERT_EQ(result.size(), 1U);
  EXPECT_TRUE(result[0].position_active);
  EXPECT_FALSE(result[0].velocity_active);
  EXPECT_FALSE(result[0].effort_active);
}

TEST(CommandMode, ValidatesOwnedInterfacesInMixedLists)
{
  const std::vector<rs::CommandModeState> current{{"joint_1", true, false, false}};
  const std::vector<std::string> start{"other/position", "joint_1/velocity"};
  const std::vector<std::string> stop{"other/effort", "joint_1/position"};
  EXPECT_TRUE(rs::validate_command_mode_switch(current, start, stop));
  const auto result = rs::command_modes_after_switch(current, start, stop);
  EXPECT_FALSE(result[0].position_active);
  EXPECT_TRUE(result[0].velocity_active);
  EXPECT_FALSE(rs::validate_command_mode_switch(
      current, {"other/position", "joint_1/velocity"}, {}));
  EXPECT_FALSE(rs::validate_command_mode_switch(
      current, {"other/position", "joint_1/temperature"}, {}));
  EXPECT_FALSE(rs::validate_command_mode_switch(
      current, {}, {"other/position", "joint_1/temperature"}));
}

TEST(CommandMode, AcceptsBroadcastSwitchForTwoComponents)
{
  const std::vector<rs::CommandModeState> first{{"motor_a", false, false, false}};
  const std::vector<rs::CommandModeState> second{{"motor_b", false, false, false}};
  const std::vector<std::string> start{"motor_a/position", "motor_b/velocity"};
  EXPECT_TRUE(rs::validate_command_mode_switch(first, start, {}));
  EXPECT_TRUE(rs::validate_command_mode_switch(second, start, {}));
  const auto first_active = rs::command_modes_after_switch(first, start, {});
  const auto second_active = rs::command_modes_after_switch(second, start, {});
  EXPECT_TRUE(first_active[0].position_active);
  EXPECT_TRUE(second_active[0].velocity_active);
  EXPECT_TRUE(rs::validate_command_mode_switch(first_active, {}, start));
  EXPECT_TRUE(rs::validate_command_mode_switch(second_active, {}, start));
  EXPECT_FALSE(rs::command_modes_after_switch(first_active, {}, start)[0].position_active);
  EXPECT_FALSE(rs::command_modes_after_switch(second_active, {}, start)[0].velocity_active);
}

TEST(CommandMode, DistinguishesJointPrefixesAndNamespacedJoints)
{
  const std::vector<rs::CommandModeState> current{
    {"joint_1", false, false, false}, {"arm/joint", false, false, false}};
  EXPECT_TRUE(rs::validate_command_mode_switch(current, {"joint_10/position"}, {}));
  EXPECT_TRUE(rs::validate_command_mode_switch(current, {"arm/joint/tool/position"}, {}));
  EXPECT_TRUE(rs::validate_command_mode_switch(current, {}, {"arm/joint/tool/position"}));
  EXPECT_TRUE(rs::validate_command_mode_switch(current, {"arm/joint/effort"}, {}));
  EXPECT_FALSE(rs::validate_command_mode_switch(current, {"arm/joint/temperature"}, {}));
}
