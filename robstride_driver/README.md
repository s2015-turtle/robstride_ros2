# robstride_driver

`robstride_driver` is the bridge-independent core library for RobStride
actuators using the documented 29-bit private CAN protocol. It provides frame
encoding and decoding, `can_msgs/msg/Frame` topic transport, motor lifecycle
commands, feedback monitoring, and Run-mode recovery.

The package does not open a SocketCAN interface and does not provide a
standalone ROS node. Applications configure the transmit and receive topics
through `robstride_ros2_control`, then connect any compatible CAN bridge. The
example package uses `ros2_socketcan`.

Public headers are installed under `robstride_driver/` and cover:

- protocol constants and command/feedback encoding;
- per-model and per-joint limits;
- CAN topic transport;
- motor activation, stop, watchdog, and feedback handling.
- lock-free snapshots of traffic counters and per-motor feedback timing.

`RobStrideDriver::metrics()` returns the measured transmit categories,
latest-command-wins replacement counts, recognized receive counts, and
per-motor feedback rate and age. The snapshot reads atomic counters and does
not acquire the transport worker or driver state mutex. When the normal ROS
topic transport is used, the same information is published once per second as
standard diagnostics on `/diagnostics`.

Most users should integrate this library through the
[`robstride_ros2_control`](https://github.com/s2015-turtle/robstride_ros2/tree/main/robstride_ros2_control)
Hardware Component. See the
[repository README](https://github.com/s2015-turtle/robstride_ros2#readme) for
supported actuators, safety behavior, configuration parameters, and
installation instructions.
