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

TEST(CommandMode, RejectsUnknownInterface)
{
  const std::vector<rs::CommandModeState> current{{"joint_1", false, false, false}};
  EXPECT_FALSE(rs::validate_command_mode_switch(current, {"joint_2/velocity"}, {}));
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
