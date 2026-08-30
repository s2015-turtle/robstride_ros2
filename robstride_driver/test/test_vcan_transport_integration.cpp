#include <linux/can.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "robstride_driver/can_transport.hpp"

using namespace std::chrono_literals;

namespace
{
class CanSocket
{
public:
  explicit CanSocket(const char * interface_name)
  {
    descriptor_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (descriptor_ < 0) {throw std::runtime_error("could not create CAN socket");}

    ifreq request{};
    std::strncpy(request.ifr_name, interface_name, IFNAMSIZ - 1);
    if (ioctl(descriptor_, SIOCGIFINDEX, &request) < 0) {
      close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error("could not resolve vcan interface");
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(descriptor_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error("could not bind CAN socket");
    }
  }

  ~CanSocket()
  {
    if (descriptor_ >= 0) {close(descriptor_);}
  }

  void send(uint32_t id, const std::array<uint8_t, 8> & data)
  {
    can_frame frame{};
    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::copy(data.begin(), data.end(), frame.data);
    if (write(descriptor_, &frame, sizeof(frame)) != sizeof(frame)) {
      throw std::runtime_error("could not send vcan frame");
    }
  }

  can_frame receive(std::chrono::milliseconds timeout)
  {
    pollfd event{descriptor_, POLLIN, 0};
    if (poll(&event, 1, static_cast<int>(timeout.count())) != 1) {
      throw std::runtime_error("timed out waiting for vcan frame");
    }
    can_frame frame{};
    if (read(descriptor_, &frame, sizeof(frame)) != sizeof(frame)) {
      throw std::runtime_error("could not read vcan frame");
    }
    return frame;
  }

private:
  int descriptor_{-1};
};

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    CanSocket socket("vcan0");
    std::mutex receive_mutex;
    std::condition_variable receive_condition;
    can_msgs::msg::Frame::ConstSharedPtr received;

    robstride_driver::CanTransportOptions options;
    options.node_name = "robstride_vcan_integration_test";
    options.transmit_topic = "/robstride_vcan/to_bus";
    options.receive_topic = "/robstride_vcan/from_bus";
    options.motor_count = 1;
    robstride_driver::CanTransport transport(
      options,
      [&](can_msgs::msg::Frame::ConstSharedPtr frame) {
        if (frame->id != 0x020001fdu) {return;}
        std::lock_guard<std::mutex> lock(receive_mutex);
        received = std::move(frame);
        receive_condition.notify_all();
      });
    transport.start();
    require(transport.wait_for_endpoints(5s), "ros2_socketcan endpoints did not appear");

    const robstride_driver::Frame outgoing{
      0x18ff00a5, {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}}};
    transport.send_transaction(outgoing);
    require(
      transport.wait_for_transaction_acknowledgements(2s),
      "outgoing ROS message was not acknowledged");
    const auto wire_frame = socket.receive(5s);
    require((wire_frame.can_id & CAN_EFF_MASK) == outgoing.id, "outgoing CAN ID changed");
    require(std::equal(outgoing.data.begin(), outgoing.data.end(), wire_frame.data),
      "outgoing CAN payload changed");

    const std::array<uint8_t, 8> incoming_data{{8, 7, 6, 5, 4, 3, 2, 1}};
    socket.send(0x020001fd, incoming_data);
    {
      std::unique_lock<std::mutex> lock(receive_mutex);
      require(receive_condition.wait_for(lock, 5s, [&]() {return received != nullptr;}),
        "incoming vcan frame did not reach the ROS callback");
      require(received->id == 0x020001fdu, "incoming CAN ID changed");
      require(std::equal(incoming_data.begin(), incoming_data.end(), received->data.begin()),
        "incoming CAN payload changed");
    }

    const auto metrics = transport.metrics();
    require(metrics.transaction_frames_transmitted == 1, "TX metric was not updated");
    transport.stop();
    rclcpp::shutdown();
    std::cout << "vcan transport integration passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
