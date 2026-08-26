# robstride_ros2

`robstride_ros2` is the aggregate installation package for this repository. It
depends on and installs:

- `robstride_driver`, the private-CAN protocol and transport library;
- `robstride_ros2_control`, the ROS 2 Hardware Component;
- `robstride_examples`, the actuator profiles and example launch files.

This package does not provide a separate node or hardware plugin. Install it
when the complete driver and examples are wanted. Projects that provide their
own robot description and CAN bridge can depend directly on
`robstride_ros2_control` instead.

See the [repository README](https://github.com/s2015-turtle/robstride_ros2#readme)
for installation, supported models, configuration parameters, controller
commands, and integration guidance.
