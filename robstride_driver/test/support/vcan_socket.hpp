#pragma once

#include <linux/can.h>

#include <array>
#include <chrono>
#include <cstdint>

namespace robstride_driver::test
{

class VcanSocket
{
public:
  explicit VcanSocket(const char * interface_name);
  ~VcanSocket();

  VcanSocket(const VcanSocket &) = delete;
  VcanSocket & operator=(const VcanSocket &) = delete;

  bool receive(can_frame & frame, std::chrono::milliseconds timeout);
  void send(uint32_t id, const std::array<uint8_t, 8> & data);

private:
  int descriptor_{-1};
};

}  // namespace robstride_driver::test
