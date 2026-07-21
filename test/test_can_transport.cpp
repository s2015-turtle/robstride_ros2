#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "robstride_ros2/can_transport.hpp"

namespace rs = robstride_ros2;

namespace
{
rs::CanTransportOptions valid_options()
{
  rs::CanTransportOptions options;
  options.node_name = "test_can_transport";
  options.motor_count = 1;
  return options;
}

const rs::CanTransport::ReceiveCallback kReceiveCallback =
  [](can_msgs::msg::Frame::ConstSharedPtr) {};
}  // namespace

TEST(CanTransport, AcceptsACompleteConfiguration)
{
  EXPECT_NO_THROW(rs::CanTransport(valid_options(), kReceiveCallback));
}

TEST(CanTransport, RejectsMissingIdentityOrMotors)
{
  auto options = valid_options();
  options.node_name.clear();
  EXPECT_THROW(rs::CanTransport(options, kReceiveCallback), std::invalid_argument);

  options = valid_options();
  options.motor_count = 0;
  EXPECT_THROW(rs::CanTransport(options, kReceiveCallback), std::invalid_argument);
}

TEST(CanTransport, RejectsInvalidQueueSettings)
{
  auto options = valid_options();
  options.receive_qos_depth = 0;
  EXPECT_THROW(rs::CanTransport(options, kReceiveCallback), std::invalid_argument);

  options = valid_options();
  options.motion_frame_lifespan = std::chrono::milliseconds(0);
  EXPECT_THROW(rs::CanTransport(options, kReceiveCallback), std::invalid_argument);
}

TEST(CanTransport, RejectsMissingReceiveCallback)
{
  EXPECT_THROW(
    rs::CanTransport(valid_options(), rs::CanTransport::ReceiveCallback{}),
    std::invalid_argument);
}
