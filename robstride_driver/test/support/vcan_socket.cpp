#include "vcan_socket.hpp"

#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace robstride_driver::test
{

VcanSocket::VcanSocket(const char * interface_name)
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

VcanSocket::~VcanSocket()
{
  if (descriptor_ >= 0) {close(descriptor_);}
}

bool VcanSocket::receive(can_frame & frame, std::chrono::milliseconds timeout)
{
  pollfd event{descriptor_, POLLIN, 0};
  const int result = poll(&event, 1, static_cast<int>(timeout.count()));
  if (result == 0) {return false;}
  if (result < 0) {throw std::runtime_error("could not poll virtual CAN interface");}
  if (read(descriptor_, &frame, sizeof(frame)) != sizeof(frame)) {
    throw std::runtime_error("could not read virtual CAN frame");
  }
  return true;
}

void VcanSocket::send(uint32_t id, const std::array<uint8_t, 8> & data)
{
  can_frame frame{};
  frame.can_id = id | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::copy(data.begin(), data.end(), frame.data);
  if (write(descriptor_, &frame, sizeof(frame)) != sizeof(frame)) {
    throw std::runtime_error("could not send virtual CAN frame");
  }
}

}  // namespace robstride_driver::test
