#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
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

template<typename Predicate>
bool wait_for_metric(Predicate predicate, std::chrono::milliseconds timeout = 1s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
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

TEST(CanTransportHealth, ReportsHealthyTransportAfterPublishing)
{
  auto capture = std::make_shared<CaptureState>();
  rs::CanTransport transport(valid_options(), kReceiveCallback, sink_for(capture));
  transport.start();
  transport.enable_active_commands();
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});

  ASSERT_TRUE(capture->wait_for_size(1));
  const auto health = transport.health(100ms);
  EXPECT_EQ(health.state, rs::CanTransportHealthState::healthy);
  EXPECT_FALSE(health.persistent);
  transport.stop();
}

TEST(CanTransportHealth, DoesNotTreatAnIdleActiveTransportAsStalled)
{
  auto capture = std::make_shared<CaptureState>();
  rs::CanTransport transport(valid_options(), kReceiveCallback, sink_for(capture));
  transport.start();
  transport.enable_active_commands();

  std::this_thread::sleep_for(10ms);
  EXPECT_EQ(transport.health(1ms).state, rs::CanTransportHealthState::healthy);
  transport.stop();
}

TEST(CanTransportHealth, ToleratesTransientEndpointLossAndRecovers)
{
  auto capture = std::make_shared<CaptureState>();
  std::atomic<bool> endpoint_available{true};
  rs::CanTransport transport(
    valid_options(), kReceiveCallback, sink_for(capture),
    rs::CanTransport::MetricsProvider{}, [&endpoint_available]() {
      return endpoint_available.load();
    });
  transport.start();
  ASSERT_TRUE(transport.wait_for_endpoints(10ms));
  transport.enable_active_commands();

  endpoint_available = false;
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});
  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.health(200ms).state ==
           rs::CanTransportHealthState::bridge_unavailable;
  }));
  const auto unavailable = transport.health(200ms);
  EXPECT_FALSE(unavailable.persistent);

  endpoint_available = true;
  transport.queue_motion_frame(0, rs::Frame{0x11, {}});
  ASSERT_TRUE(capture->wait_for_size(1));
  EXPECT_EQ(transport.health(200ms).state, rs::CanTransportHealthState::healthy);
  transport.stop();
}

TEST(CanTransportHealth, ReportsPersistentEndpointLoss)
{
  auto capture = std::make_shared<CaptureState>();
  rs::CanTransport transport(
    valid_options(), kReceiveCallback, sink_for(capture),
    rs::CanTransport::MetricsProvider{}, []() {return false;});
  transport.start();
  EXPECT_FALSE(transport.wait_for_endpoints(10ms));
  transport.enable_active_commands();
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});

  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.health(10ms).persistent;
  }));
  const auto health = transport.health(10ms);
  EXPECT_EQ(health.state, rs::CanTransportHealthState::bridge_unavailable);
  EXPECT_TRUE(health.persistent);
  transport.stop();
}

TEST(CanTransportHealth, ReportsAStalledTransmitWorker)
{
  auto capture = std::make_shared<CaptureState>();
  capture->blocking_id = 0x10;
  rs::CanTransport transport(valid_options(), kReceiveCallback, sink_for(capture));
  CaptureReleaseGuard release_guard(capture);
  transport.start();
  transport.enable_active_commands();
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});

  ASSERT_TRUE(capture->wait_until_blocked());
  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.health(10ms).state == rs::CanTransportHealthState::transmit_stalled;
  }));
  EXPECT_TRUE(transport.health(10ms).persistent);
  capture->release();
  transport.stop();
}

TEST(CanTransportHealth, ReportsAWorkerStoppedByAnException)
{
  rs::CanTransport transport(
    valid_options(), kReceiveCallback,
    [](const rs::Frame &) {throw std::runtime_error("simulated transmit failure");});
  transport.start();
  transport.enable_active_commands();
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});

  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.health(1s).state == rs::CanTransportHealthState::worker_stopped;
  }));
  EXPECT_TRUE(transport.health(1s).persistent);
  transport.stop();
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
  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.metrics().motion_frames_transmitted == 1;
  }));
  const auto metrics = transport.metrics();
  EXPECT_EQ(metrics.recovery_frames_transmitted, 1u);
  EXPECT_EQ(metrics.motion_frames_transmitted, 1u);
  EXPECT_EQ(metrics.transmitted_frames(), 2u);
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
  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.metrics().transaction_frames_transmitted == 3;
  }));
  const auto metrics = transport.metrics();
  EXPECT_EQ(metrics.transaction_frames_transmitted, 3u);
  EXPECT_GT(metrics.transmit_rate_hz(), 0.0);
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
  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.metrics().motion_frames_transmitted == 1;
  }));
  const auto metrics = transport.metrics();
  EXPECT_EQ(metrics.transaction_frames_transmitted, 1u);
  EXPECT_EQ(metrics.motion_frames_transmitted, 1u);
  EXPECT_EQ(metrics.motion_frames_coalesced, 1u);
  transport.stop();
}

TEST(CanTransport, CoalescesAMultiMotorCommandBatch)
{
  auto capture = std::make_shared<CaptureState>();
  capture->blocking_id = 0x40;
  rs::CanTransport transport(valid_options(3), kReceiveCallback, sink_for(capture));
  CaptureReleaseGuard release_guard(capture);
  transport.start();
  transport.enable_active_commands();

  transport.send_transaction(rs::Frame{0x40, {}});
  ASSERT_TRUE(capture->wait_until_blocked());
  transport.queue_motion_frames({
    {0, rs::Frame{0x10, {}}},
    {1, rs::Frame{0x11, {}}},
    {2, rs::Frame{0x12, {}}}});
  transport.queue_motion_frames({
    {0, rs::Frame{0x20, {}}},
    {1, rs::Frame{0x21, {}}},
    {2, rs::Frame{0x22, {}}}});
  capture->release();

  ASSERT_TRUE(capture->wait_for_size(4));
  const auto frames = capture->snapshot();
  ASSERT_EQ(frames.size(), 4u);
  EXPECT_EQ(frames[0].id, 0x40u);
  EXPECT_EQ(frames[1].id, 0x20u);
  EXPECT_EQ(frames[2].id, 0x21u);
  EXPECT_EQ(frames[3].id, 0x22u);
  ASSERT_TRUE(wait_for_metric([&transport]() {
    return transport.metrics().motion_frames_transmitted == 3;
  }));
  EXPECT_EQ(transport.metrics().motion_frames_coalesced, 3u);
  transport.stop();
}

TEST(CanTransport, AppliesMultiMotorRecoveryUpdatesAsOneBatch)
{
  auto capture = std::make_shared<CaptureState>();
  rs::CanTransport transport(valid_options(2), kReceiveCallback, sink_for(capture));
  transport.start();
  transport.enable_active_commands();

  transport.apply_recovery_updates({
    {0, rs::Frame{0x30, {}}},
    {1, rs::Frame{0x31, {}}}});
  transport.queue_motion_frames({
    {0, rs::Frame{0x10, {}}},
    {1, rs::Frame{0x11, {}}}});
  ASSERT_TRUE(capture->wait_for_size(2));
  EXPECT_EQ(capture->snapshot()[0].id, 0x30u);
  EXPECT_EQ(capture->snapshot()[1].id, 0x31u);

  transport.apply_recovery_updates({
    {0, std::nullopt},
    {1, std::nullopt}});
  ASSERT_TRUE(capture->wait_for_size(4));
  EXPECT_EQ(capture->snapshot()[2].id, 0x10u);
  EXPECT_EQ(capture->snapshot()[3].id, 0x11u);
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

TEST(CanTransport, StopsWhileBatchesAreBeingProduced)
{
  auto capture = std::make_shared<CaptureState>();
  rs::CanTransport transport(valid_options(2), kReceiveCallback, sink_for(capture));
  transport.start();
  transport.enable_active_commands();

  auto producer = std::async(std::launch::async, [&transport]() {
      for (uint32_t sequence = 0; sequence < 200; ++sequence) {
        transport.queue_motion_frames({
          {0, rs::Frame{0x100 + sequence, {}}},
          {1, rs::Frame{0x200 + sequence, {}}}});
        if (sequence % 4 == 0) {
          transport.apply_recovery_updates({{0, rs::Frame{0x300 + sequence, {}}}});
        } else if (sequence % 4 == 1) {
          transport.apply_recovery_updates({{0, std::nullopt}});
        }
      }
    });
  auto stop = std::async(std::launch::async, [&transport]() {transport.stop();});

  ASSERT_EQ(producer.wait_for(5s), std::future_status::ready);
  producer.get();
  ASSERT_EQ(stop.wait_for(5s), std::future_status::ready);
  stop.get();
}
