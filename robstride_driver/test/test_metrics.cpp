#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <thread>

#include "robstride_driver/metrics.hpp"

namespace rs = robstride_driver;

namespace
{
bool same_sample(const rs::MotorFeedbackSample & lhs, const rs::MotorFeedbackSample & rhs)
{
  return lhs.count == rhs.count && lhs.last_received_at_ns == rhs.last_received_at_ns &&
         lhs.maximum_gap_ns == rhs.maximum_gap_ns && lhs.mode == rhs.mode &&
         lhs.fault_flags == rhs.fault_flags && lhs.temperature == rhs.temperature;
}
}  // namespace

TEST(MotorDiagnostics, DecodesFeedbackFaultFlags)
{
  EXPECT_EQ(rs::motor_fault_summary(0), "none");
  EXPECT_EQ(rs::motor_fault_summary(0x01), "undervoltage");
  EXPECT_EQ(
    rs::motor_fault_summary(0x3f),
    "undervoltage, overcurrent, over-temperature, encoder fault, stall overload, "
    "encoder uncalibrated");
}

TEST(MotorDiagnostics, NamesMotorModesAndTransportHealth)
{
  EXPECT_STREQ(rs::motor_mode_name(0), "Reset");
  EXPECT_STREQ(rs::motor_mode_name(1), "Calibration");
  EXPECT_STREQ(rs::motor_mode_name(2), "Run");
  EXPECT_STREQ(rs::motor_mode_name(3), "Unknown");
  EXPECT_STREQ(
    rs::transport_health_name(rs::CanTransportHealthState::transmit_stalled),
    "transmit stalled");
}

TEST(MotorDiagnostics, ReadsAConsistentSnapshotDuringConcurrentUpdates)
{
  rs::AtomicMotorFeedback feedback;
  const rs::MotorFeedbackSample first{1, 100, 10, 0, 0x01, 20.0};
  const rs::MotorFeedbackSample second{2, 200, 20, 2, 0x20, 40.0};
  feedback.store(first);

  std::atomic<bool> finished{false};
  std::atomic<bool> inconsistent{false};
  std::thread writer([&]() {
    for (size_t iteration = 0; iteration < 100000; ++iteration) {
      feedback.store((iteration & 1u) == 0u ? second : first);
    }
    finished = true;
  });
  while (!finished) {
    const auto snapshot = feedback.load();
    if (!same_sample(snapshot, first) && !same_sample(snapshot, second)) {
      inconsistent = true;
      break;
    }
  }
  writer.join();
  EXPECT_FALSE(inconsistent);
}
