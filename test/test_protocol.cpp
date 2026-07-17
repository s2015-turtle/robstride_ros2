#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "robstride_ros2/protocol.hpp"

namespace rs = robstride_ros2;

constexpr double kPi = 3.14159265358979323846;
constexpr rs::Limits kRs02{-4.0 * kPi, 4.0 * kPi, -44.0, 44.0, -17.0, 17.0, 500.0, 5.0};

TEST(Protocol, EncodeEndpointsAndCenter)
{
  EXPECT_EQ(rs::encode_u16(-17.0, -17.0, 17.0), 0);
  EXPECT_EQ(rs::encode_u16(17.0, -17.0, 17.0), 65535);
  EXPECT_EQ(rs::encode_u16(0.0, -17.0, 17.0), 32768);
}

TEST(Protocol, MotionCommandUsesExtendedIdFieldsAndBigEndianPayload)
{
  const auto frame = rs::make_motion_command(2, kRs02, 0.0, 44.0, -17.0, 500.0, 5.0);
  EXPECT_EQ((frame.id >> 24) & 0x1f, 1u);
  EXPECT_EQ(frame.id & 0xff, 2u);
  EXPECT_EQ((frame.id >> 8) & 0xffff, 0u);
  EXPECT_EQ(frame.data[0], 0x80);
  EXPECT_EQ(frame.data[1], 0x00);
  EXPECT_EQ(frame.data[2], 0xff);
  EXPECT_EQ(frame.data[3], 0xff);
  EXPECT_EQ(frame.data[4], 0xff);
  EXPECT_EQ(frame.data[5], 0xff);
  EXPECT_EQ(frame.data[6], 0xff);
  EXPECT_EQ(frame.data[7], 0xff);
}

TEST(Protocol, LifecycleAndParameterFrames)
{
  EXPECT_EQ(rs::make_enable(3, 0xfd).id, 0x0300fd03u);
  EXPECT_EQ(rs::make_stop(3, 0xfd, true).data[0], 1u);
  EXPECT_EQ(rs::make_set_zero(3, 0xfd).data[0], 1u);
  const auto mode = rs::make_write_u8(3, 0xfd, rs::kIndexRunMode, 0);
  EXPECT_EQ(mode.id, 0x1200fd03u);
  EXPECT_EQ(mode.data[0], 0x05);
  EXPECT_EQ(mode.data[1], 0x70);
}

TEST(Protocol, DecodesFeedbackAndRejectsWrongHost)
{
  std::array<uint8_t, 8> data{0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2c};
  const uint32_t id = (2u << 24) | ((0x8000u | 2u) << 8) | 0xfdu;
  const auto feedback = rs::decode_feedback(id, data, 8, true, 0xfd, kRs02);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->motor_id, 2u);
  EXPECT_EQ(feedback->mode, 2u);
  EXPECT_NEAR(feedback->position, 0.0, 0.001);
  EXPECT_NEAR(feedback->velocity, 0.0, 0.01);
  EXPECT_NEAR(feedback->effort, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(feedback->temperature, 30.0);
  EXPECT_FALSE(rs::decode_feedback(id, data, 8, true, 0xfe, kRs02).has_value());
  EXPECT_FALSE(rs::decode_feedback(id, data, 7, true, 0xfd, kRs02).has_value());
}
