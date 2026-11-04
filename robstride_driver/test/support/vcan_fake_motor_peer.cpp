#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "fake_robstride_motor.hpp"

namespace rs = robstride_driver;

namespace
{
std::string parameter_json(const rs::test::FakeRobStrideMotor & motor, uint16_t index)
{
  // Before controller_manager starts, no parameter write has reached the peer.
  try {return std::to_string(motor.parameter(index));}
  catch (const std::runtime_error &) {return "null";}
}
}  // namespace

// A line-based test control channel keeps observations independent of DDS.
// All motor commands are received from vcan, never from this stdin channel.
int main()
{
  try {
    constexpr double pi = 3.14159265358979323846;
    const rs::Limits limits{-4 * pi, 4 * pi, -50, 50, -6, 6, -6, 6, 500, 5};
    rs::test::FakeRobStrideMotor motor({"vcan0", 1, 0xfd});
    std::string line;
    while (std::getline(std::cin, line)) {
      motor.check_worker();
      std::istringstream input(line);
      std::string command;
      input >> command;
      if (command == "quit") {return 0;}
      if (command == "position") {
        double position = 0;
        if (!(input >> position) || !std::isfinite(position)) {
          throw std::runtime_error("invalid injected position");
        }
        motor.report_position(position, limits);
      } else if (command != "snapshot") {
        throw std::runtime_error("unknown peer command");
      }
      const auto frame = motor.last_motion_frame();
      std::cout << "{\"mode\":" << static_cast<unsigned>(motor.mode())
                << ",\"enables\":" << motor.enable_count()
                << ",\"motions\":" << motor.motion_count()
                << ",\"stops\":" << motor.stop_count()
                << ",\"watchdog\":" << parameter_json(motor, rs::kIndexCanTimeout)
                << ",\"run_mode\":" << parameter_json(motor, rs::kIndexRunMode)
                << ",\"id\":" << (frame ? frame->id : 0) << ",\"data\":[";
      if (frame) {
        for (size_t index = 0; index < frame->data.size(); ++index) {
          if (index != 0) {std::cout << ',';}
          std::cout << static_cast<unsigned>(frame->data[index]);
        }
      }
      std::cout << "]}" << std::endl;
    }
  } catch (const std::exception & error) {
    std::cerr << "vcan fake motor peer: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
