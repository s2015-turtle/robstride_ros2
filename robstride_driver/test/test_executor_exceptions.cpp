#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "robstride_driver/can_transport.hpp"

namespace rs = robstride_driver;
using namespace std::chrono_literals;

namespace
{
// Every test owns its ROS context: shutdown tests cannot invalidate another test's nodes.
// There is deliberately no auxiliary spin thread whose exception could hide a regression.
class ExecutorExceptions : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static std::atomic<unsigned int> sequence{0};
    const auto suffix = std::to_string(sequence.fetch_add(1));
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    options_.node_name = "executor_exception_transport_" + suffix;
    options_.transmit_topic = "/executor_exception_" + suffix + "/tx";
    options_.receive_topic = "/executor_exception_" + suffix + "/rx";
    options_.motor_count = 1;
    options_.context = context_;
    peer_ = std::make_shared<rclcpp::Node>(
      "executor_exception_peer_" + suffix, rclcpp::NodeOptions().context(context_));
    receive_publisher_ = peer_->create_publisher<can_msgs::msg::Frame>(
      options_.receive_topic, rclcpp::QoS(32).reliable());
    transmit_subscription_ = peer_->create_subscription<can_msgs::msg::Frame>(
      options_.transmit_topic, rclcpp::QoS(32).reliable(),
      [](can_msgs::msg::Frame::ConstSharedPtr) {});
    const auto hardware_id = options_.transmit_topic + " -> " + options_.receive_topic;
    diagnostics_subscription_ = peer_->create_subscription<
      diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10).reliable(),
      [this, hardware_id](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        for (const auto & status : message->status) {
          if (status.hardware_id == hardware_id &&
            status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
            status.message == "executor failed")
          {
            error_diagnostic_received_ = true;
          }
        }
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    peer_executor_ =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    peer_executor_->add_node(peer_);
  }

  void TearDown() override
  {
    peer_executor_.reset();
    diagnostics_subscription_.reset();
    transmit_subscription_.reset();
    receive_publisher_.reset();
    peer_.reset();
    if (context_ && context_->is_valid()) {context_->shutdown("test complete");}
    context_.reset();
  }

  template<typename Predicate>
  bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      if (predicate()) {return true;}
      std::this_thread::sleep_for(5ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }

  bool await_failure(rs::CanTransport & transport)
  {
    return wait_until([&transport]() {
      return transport.health(1s).state == rs::CanTransportHealthState::executor_failed;
    });
  }

  bool await_diagnostics_endpoint()
  {
    return wait_until([this]() {return diagnostics_subscription_->get_publisher_count() > 0;});
  }

  bool await_error_diagnostic()
  {
    return wait_until([this]() {
      peer_executor_->spin_some();
      return error_diagnostic_received_;
    });
  }

  void publish_receive_frame()
  {
    can_msgs::msg::Frame frame;
    frame.id = 0x02000101;
    frame.is_extended = true;
    frame.dlc = 8;
    receive_publisher_->publish(frame);
  }

  bool error_diagnostic_received_{false};
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> peer_executor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Context::SharedPtr context_;
  rs::CanTransportOptions options_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr receive_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr transmit_subscription_;
};

class ReceiveExceptions : public ExecutorExceptions,
  public ::testing::WithParamInterface<bool> {};

TEST_P(ReceiveExceptions, ContainsCallbackExceptionAndLatchesHealthThroughStop)
{
  const bool standard_exception = GetParam();
  std::atomic<unsigned int> callbacks{0};
  rs::CanTransport transport(options_, [&](can_msgs::msg::Frame::ConstSharedPtr) {
    ++callbacks;
    if (standard_exception) {throw std::runtime_error("injected receive failure");}
    throw 17;
  });
  transport.start();
  ASSERT_TRUE(transport.wait_for_endpoints(5s));
  transport.enable_active_commands();
  ASSERT_TRUE(await_diagnostics_endpoint());
  publish_receive_frame();

  ASSERT_TRUE(await_failure(transport));
  ASSERT_TRUE(await_error_diagnostic());
  EXPECT_EQ(callbacks.load(), 1u);
  EXPECT_TRUE(transport.health(1h).persistent);
  const auto transmitted = transport.metrics().transmitted_frames();
  transport.enable_active_commands();
  transport.queue_motion_frame(0, rs::Frame{0x10, {}});
  transport.send_transaction(rs::Frame{0x20, {}});
  EXPECT_FALSE(transport.wait_for_transaction_acknowledgements(100ms));
  EXPECT_NO_THROW(transport.stop());
  EXPECT_EQ(transport.metrics().transmitted_frames(), transmitted);
  EXPECT_NO_THROW(transport.stop());
  EXPECT_EQ(transport.health(1h).state, rs::CanTransportHealthState::executor_failed);
}

INSTANTIATE_TEST_SUITE_P(StandardAndUnknown, ReceiveExceptions, ::testing::Bool());

TEST_F(ExecutorExceptions, RestartsDirectlyAfterFailureAndReceivesAgain)
{
  std::atomic<bool> should_throw{true};
  std::atomic<unsigned int> callbacks{0};
  rs::CanTransport transport(options_, [&](can_msgs::msg::Frame::ConstSharedPtr) {
    ++callbacks;
    if (should_throw.load()) {throw std::runtime_error("first run fails");}
  });
  transport.start();
  ASSERT_TRUE(transport.wait_for_endpoints(5s));
  publish_receive_frame();
  ASSERT_TRUE(await_failure(transport));

  should_throw = false;
  // start() must join the previous failed threads before replacing their std::threads.
  ASSERT_NO_THROW(transport.start());
  ASSERT_TRUE(transport.wait_for_endpoints(5s));
  publish_receive_frame();
  ASSERT_TRUE(wait_until([&]() {return callbacks.load() >= 2;}));
  EXPECT_EQ(transport.health(1s).state, rs::CanTransportHealthState::healthy);
  EXPECT_FALSE(transport.health(1s).persistent);
  transport.send_transaction(rs::Frame{0x20, {}});
  ASSERT_TRUE(wait_until([&]() {
    return transport.metrics().transaction_frames_transmitted == 1;
  }));
  EXPECT_NO_THROW(transport.stop());
}

TEST_F(ExecutorExceptions, DestructionJoinsFailedExecutorWithoutExplicitStop)
{
  auto transport = std::make_unique<rs::CanTransport>(options_,
    [](can_msgs::msg::Frame::ConstSharedPtr) {throw std::runtime_error("destruction test");});
  transport->start();
  ASSERT_TRUE(transport->wait_for_endpoints(5s));
  publish_receive_frame();
  ASSERT_TRUE(await_failure(*transport));
  EXPECT_NO_THROW(transport.reset());
}

class DiagnosticsExceptions : public ExecutorExceptions,
  public ::testing::WithParamInterface<bool> {};

TEST_P(DiagnosticsExceptions, ContainsTimerProviderExceptionAndCanRestart)
{
  const bool standard_exception = GetParam();
  std::atomic<bool> should_throw{false};
  std::atomic<unsigned int> samples{0};
  rs::CanTransport transport(options_, [](can_msgs::msg::Frame::ConstSharedPtr) {},
    rs::CanTransport::FrameSink{}, [&]() -> rs::DriverMetrics {
      ++samples;
      if (should_throw.load()) {
        if (standard_exception) {throw std::runtime_error("injected diagnostics failure");}
        throw 23;
      }
      return rs::DriverMetrics{};
    });
  transport.start();
  ASSERT_TRUE(transport.wait_for_endpoints(5s));
  ASSERT_TRUE(await_diagnostics_endpoint());
  should_throw = true;
  ASSERT_TRUE(await_failure(transport));
  // The fallback must not invoke the same throwing metrics provider recursively.
  ASSERT_TRUE(await_error_diagnostic());
  EXPECT_GE(samples.load(), 1u);
  EXPECT_TRUE(transport.health(1h).persistent);
  EXPECT_NO_THROW(transport.stop());
  EXPECT_EQ(transport.health(1h).state, rs::CanTransportHealthState::executor_failed);

  const auto failed_samples = samples.load();
  should_throw = false;
  ASSERT_NO_THROW(transport.start());
  ASSERT_TRUE(transport.wait_for_endpoints(5s));
  ASSERT_TRUE(wait_until([&]() {return samples.load() > failed_samples;}));
  EXPECT_EQ(transport.health(1s).state, rs::CanTransportHealthState::healthy);
  EXPECT_NO_THROW(transport.stop());
}

INSTANTIATE_TEST_SUITE_P(StandardAndUnknown, DiagnosticsExceptions, ::testing::Bool());

TEST_F(ExecutorExceptions, ContextShutdownIsAnExpectedStopNotExecutorFailure)
{
  rs::CanTransport transport(options_, [](can_msgs::msg::Frame::ConstSharedPtr) {});
  transport.start();
  ASSERT_TRUE(transport.wait_for_endpoints(5s));
  ASSERT_TRUE(context_->shutdown("intentional context shutdown"));
  ASSERT_TRUE(wait_until([&]() {
    return transport.health(1s).state == rs::CanTransportHealthState::context_shutdown;
  }));
  EXPECT_NO_THROW(transport.stop());
  EXPECT_NO_THROW(transport.stop());
  EXPECT_EQ(transport.health(1s).state, rs::CanTransportHealthState::context_shutdown);
}

TEST_F(ExecutorExceptions, CleansUpPartiallyConstructedRosResourcesOnStartupFailure)
{
  // The node and transmit publisher are created before this invalid subscription topic.
  options_.receive_topic = "/invalid topic";
  rs::CanTransport transport(options_, [](can_msgs::msg::Frame::ConstSharedPtr) {});
  EXPECT_THROW(transport.start(), std::exception);
  EXPECT_EQ(transport.health(1s).state, rs::CanTransportHealthState::executor_failed);
  EXPECT_NO_THROW(transport.stop());
  EXPECT_THROW(transport.start(), std::exception);
  // Destructor must also tolerate a second partially initialized startup failure.
}

TEST_F(ExecutorExceptions, FailedRestartWithShutdownContextRetainsFailureAndIsDestructible)
{
  rs::CanTransport transport(options_, [](can_msgs::msg::Frame::ConstSharedPtr) {
    throw std::runtime_error("failure before failed restart");
  });
  transport.start();
  ASSERT_TRUE(transport.wait_for_endpoints(5s));
  publish_receive_frame();
  ASSERT_TRUE(await_failure(transport));
  ASSERT_TRUE(context_->shutdown("make restart fail"));
  EXPECT_THROW(transport.start(), std::exception);
  EXPECT_EQ(transport.health(1h).state, rs::CanTransportHealthState::executor_failed);
  EXPECT_NO_THROW(transport.stop());
}
}  // namespace
