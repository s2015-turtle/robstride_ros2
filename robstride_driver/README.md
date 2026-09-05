# robstride_driver

The motor profile registry in `src/motor_profile.cpp` owns the RS00–RS06 and
EL05 protocol ranges. `motor_profile()` exposes these to configuration clients;
unknown names are rejected. These ranges retain the values checked against the
RobStride English manuals dated July 13, 2026, linked in the source and repository
README. They describe CAN normalization, not recommended operating limits.

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
latest-command-wins motion-command replacement count, recognized receive
counts, transport health, and per-motor mode, temperature, decoded faults,
recovery activity, feedback rate, and age. The snapshot reads a consistent
atomic status sample per motor and does not acquire the transport worker or
driver state mutex. When the normal ROS topic transport is used, the same
information is published once per second as standard diagnostics on
`/diagnostics`.

While active, the driver distinguishes temporary topic endpoint loss from a
persistent transmit failure. Persistent endpoint loss, a stalled sender, or an
unexpectedly stopped worker is propagated through the Hardware Component's
`write()` result. Motor feedback timeout remains an independent end-to-end
check because DDS publication alone cannot prove physical CAN delivery or motor
execution.

Most users should integrate this library through the
[`robstride_ros2_control`](https://github.com/s2015-turtle/robstride_ros2/tree/main/robstride_ros2_control)
Hardware Component. See the
[repository README](https://github.com/s2015-turtle/robstride_ros2#readme) for
supported actuators, safety behavior, configuration parameters, and
installation instructions.
