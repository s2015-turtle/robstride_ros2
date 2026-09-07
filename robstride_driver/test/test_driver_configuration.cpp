#include <gtest/gtest.h>

#include <limits>

#include "robstride_driver/driver.hpp"

namespace rs = robstride_driver;

TEST(DriverConfiguration, NeutralRangesAreValidatedAtPublicEntryPoint)
{
  rs::DriverConfiguration base;
  base.settings.transport.node_name = "neutral_range_test";
  rs::JointData joint;
  joint.name = "motor1";
  joint.can_id = 1;
  joint.can_timeout_ticks = 4000;
  joint.limits = {-12.0, 12.0, -50.0, 50.0, -6.0, 6.0, -6.0, 6.0, 500.0, 5.0};
  // A position range excluding zero remains valid.
  joint.command_limits = {1.0, 2.0, -5.0, 5.0, -3.0, 3.0};
  base.joints.push_back(joint);

  for (int field = 0; field < 5; ++field) {
    for (const double minimum : {-2.0, 0.0, 1.0}) {
      for (const double maximum : {-1.0, 0.0, 2.0}) {
        auto config = base;
        auto & item = config.joints.front();
        double * minima[] = {&item.command_limits.velocity_min, &item.command_limits.effort_min,
          &item.limits.velocity_min, &item.limits.effort_min, &item.limits.effort_wire_min};
        double * maxima[] = {&item.command_limits.velocity_max, &item.command_limits.effort_max,
          &item.limits.velocity_max, &item.limits.effort_max, &item.limits.effort_wire_max};
        *minima[field] = minimum;
        *maxima[field] = maximum;
        SCOPED_TRACE(::testing::Message() << field << ": " << minimum << ", " << maximum);
        rs::RobStrideDriver driver(rclcpp::get_logger("neutral_range_test"));
        EXPECT_EQ(driver.initialize(config), minimum < maximum && minimum <= 0 && maximum >= 0);
        *minima[field] = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(driver.initialize(config));
        *minima[field] = -std::numeric_limits<double>::infinity();
        EXPECT_FALSE(driver.initialize(config));
      }
    }
  }
}
