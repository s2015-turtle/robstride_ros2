#include <linux/can.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "robstride_driver/can_transport.hpp"
#include "support/vcan_socket.hpp"

using namespace std::chrono_literals;
namespace rs = robstride_driver;
namespace test = robstride_driver::test;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    test::VcanSocket socket("vcan0");
    std::mutex receive_mutex;
    std::condition_variable receive_condition;
    can_msgs::msg::Frame::ConstSharedPtr received;

    rs::CanTransportOptions options;
    options.node_name = "robstride_vcan_integration_test";
    options.transmit_topic = "/robstride_vcan/to_bus";
    options.receive_topic = "/robstride_vcan/from_bus";
    options.motor_count = 1;
    rs::CanTransport transport(
      options,
      [&](can_msgs::msg::Frame::ConstSharedPtr frame) {
        if (frame->id != 0x020001fdu) {return;}
        std::lock_guard<std::mutex> lock(receive_mutex);
        received = std::move(frame);
        receive_condition.notify_all();
      });
    transport.start();
    require(transport.wait_for_endpoints(5s), "ros2_socketcan endpoints did not appear");
    std::this_thread::sleep_for(500ms);

    const rs::Frame outgoing{
      0x01020304u, {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17}};
    transport.send_transaction(outgoing);
    require(transport.wait_for_transaction_acknowledgements(5s),
      "outgoing ROS frame was not acknowledged");
    can_frame bus_frame{};
    require(socket.receive(bus_frame, 5s), "outgoing frame did not reach vcan");
    require((bus_frame.can_id & CAN_EFF_MASK) == outgoing.id, "outgoing CAN ID changed");
    require((bus_frame.can_id & CAN_EFF_FLAG) != 0, "outgoing frame was not extended");
    require(bus_frame.can_dlc == 8, "outgoing DLC changed");
    for (size_t i = 0; i < outgoing.data.size(); ++i) {
      require(bus_frame.data[i] == outgoing.data[i], "outgoing payload changed");
    }

    const std::array<uint8_t, 8> incoming{
      0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
    socket.send(0x020001fdu, incoming);
    {
      std::unique_lock<std::mutex> lock(receive_mutex);
      require(receive_condition.wait_for(lock, 5s, [&]() {return static_cast<bool>(received);}),
        "incoming vcan frame did not reach the ROS callback");
    }
    require(received->is_extended, "incoming frame lost the extended flag");
    require(received->dlc == 8, "incoming DLC changed");
    require(received->data == incoming, "incoming payload changed");

    transport.stop();
    rclcpp::shutdown();
    std::cout << "vcan transport integration passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "vcan transport integration failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
