#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "robstride_driver/can_transport.hpp"

namespace rs = robstride_driver;
using namespace std::chrono_literals;

namespace
{
rs::CanTransportOptions valid_options(size_t motor_count = 1)
{
  rs::CanTransportOptions options;
  options.node_name = "test_can_transport";
  options.motor_count = motor_count;
  return options;
}

const rs::CanTransport::ReceiveCallback kReceiveCallback =
  [](can_msgs::msg::Frame::ConstSharedPtr) {};

struct CaptureState
{
  void capture(const rs::Frame & frame)
  {
    std::unique_lock<std::mutex> lock(mutex);
    frames.push_back(frame);
    condition.notify_all();
    if (blocking_id && frame.id == *blocking_id) {
      blocked = true;
      condition.notify_all();
      (void)condition.wait_for(lock, 2s, [this]() {return released;});
    }
  }

  bool wait_for_size(size_t size, std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, timeout, [this, size]() {return frames.size() >= size;});
  }

  bool wait_until_blocked()
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, 1s, [this]() {return blocked;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    condition.notify_all();
  }

  std::vector<rs::Frame> snapshot()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return frames;
  }

  std::mutex mutex;
  std::condition_variable condition;
  std::vector<rs::Frame> frames;
  std::optional<uint32_t> blocking_id;
  bool blocked{false};
  bool released{false};
};

rs::CanTransport::FrameSink sink_for(const std::shared_ptr<CaptureState> & capture)
{
  return [capture](const rs::Frame & frame) {capture->capture(frame);};
}

struct CaptureReleaseGuard
{
  explicit CaptureReleaseGuard(std::shared_ptr<CaptureState> capture_state)
  : capture(std::move(capture_state)) {}

  ~CaptureReleaseGuard() {capture->release();}

  std::shared_ptr<CaptureState> capture;
};
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

TEST(CanTransport, RejectsInvalidReceiveQueueDepth)
{
  auto options = valid_options();
  options.receive_qos_depth = 0;
  EXPECT_THROW(rs::CanTransport(options, kReceiveCallback), std::invalid_argument);
}

TEST(CanTransport, RejectsMissingReceiveCallback)
{
  EXPECT_THROW(
    rs::CanTransport(valid_options(), rs::CanTransport::ReceiveCallback{}),
    std::invalid_argument);
}

TEST(CanTransport, HoldsMotionUntilRecoveryCompletes)
{
  auto capture = std::make_shared<CaptureState>();
  rs::CanTransport transport(valid_options(), kReceiveCallback, sink_for(capture));
  transport.start();
  transport.enable_active_commands();

  transport.queue_recovery_frame(0, rs::Frame{0x30, {}});
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});
  ASSERT_TRUE(capture->wait_for_size(1));
  EXPECT_EQ(capture->snapshot()[0].id, 0x30u);
  EXPECT_EQ(capture->snapshot().size(), 1u);

  transport.complete_recovery(0);
  ASSERT_TRUE(capture->wait_for_size(2));
  EXPECT_EQ(capture->snapshot()[1].id, 0x10u);
  transport.stop();
}

TEST(CanTransport, PreservesTransactionOrderOnTheSingleWriter)
{
  auto capture = std::make_shared<CaptureState>();
  rs::CanTransport transport(valid_options(), kReceiveCallback, sink_for(capture));
  transport.start();

  transport.send_transaction(rs::Frame{0x20, {}});
  transport.send_transaction(rs::Frame{0x21, {}});
  transport.send_transaction(rs::Frame{0x22, {}});
  ASSERT_TRUE(transport.wait_for_transaction_acknowledgements(1s));

  const auto frames = capture->snapshot();
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[0].id, 0x20u);
  EXPECT_EQ(frames[1].id, 0x21u);
  EXPECT_EQ(frames[2].id, 0x22u);
  transport.stop();
}

TEST(CanTransport, ReplacesAnUnsentMotionFrameWithTheLatestValue)
{
  auto capture = std::make_shared<CaptureState>();
  capture->blocking_id = 0x40;
  rs::CanTransport transport(valid_options(), kReceiveCallback, sink_for(capture));
  CaptureReleaseGuard release_guard(capture);
  transport.start();
  transport.enable_active_commands();

  transport.send_transaction(rs::Frame{0x40, {}});
  ASSERT_TRUE(capture->wait_until_blocked());
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});
  transport.queue_motion_frame(0, rs::Frame{0x11, {}});
  capture->release();

  ASSERT_TRUE(capture->wait_for_size(2));
  const auto frames = capture->snapshot();
  EXPECT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id, 0x40u);
  EXPECT_EQ(frames[1].id, 0x11u);
  transport.stop();
}

TEST(CanTransport, RejectsExtractedFramesFromAnOlderActivation)
{
  auto capture = std::make_shared<CaptureState>();
  capture->blocking_id = 0x10;
  rs::CanTransport transport(valid_options(2), kReceiveCallback, sink_for(capture));
  CaptureReleaseGuard release_guard(capture);
  transport.start();
  transport.enable_active_commands();

  transport.queue_motion_frame(0, rs::Frame{0x10, {}});
  transport.queue_motion_frame(1, rs::Frame{0x11, {}});
  ASSERT_TRUE(capture->wait_until_blocked());

  auto disable = std::async(std::launch::async, [&transport]() {
      transport.disable_active_commands();
    });
  EXPECT_EQ(disable.wait_for(20ms), std::future_status::timeout);
  capture->release();
  disable.get();
  transport.enable_active_commands();

  EXPECT_FALSE(capture->wait_for_size(2));
  const auto frames = capture->snapshot();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].id, 0x10u);
  transport.stop();
}

TEST(CanTransport, DrainsTransactionsWhenStopped)
{
  auto capture = std::make_shared<CaptureState>();
  capture->blocking_id = 0x20;
  rs::CanTransport transport(valid_options(), kReceiveCallback, sink_for(capture));
  CaptureReleaseGuard release_guard(capture);
  transport.start();

  transport.send_transaction(rs::Frame{0x20, {}});
  ASSERT_TRUE(capture->wait_until_blocked());
  transport.send_transaction(rs::Frame{0x21, {}});

  auto stop = std::async(std::launch::async, [&transport]() {transport.stop();});
  EXPECT_EQ(stop.wait_for(20ms), std::future_status::timeout);
  capture->release();
  ASSERT_TRUE(capture->wait_for_size(2, 5s));
  ASSERT_EQ(stop.wait_for(5s), std::future_status::ready);
  stop.get();

  const auto frames = capture->snapshot();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id, 0x20u);
  EXPECT_EQ(frames[1].id, 0x21u);
}
