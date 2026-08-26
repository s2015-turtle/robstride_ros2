# robstride_ros2_control

`robstride_ros2_control` exposes RobStride actuators as a ROS 2
`hardware_interface::SystemInterface`. One component can manage multiple motor
IDs on one CAN bus, and each joint can independently use position, velocity, or
effort commands.

Use the following plugin identifier in a `ros2_control` robot description:

```xml
<hardware>
  <plugin>robstride_ros2/RobStrideSystem</plugin>
</hardware>
```

Every joint exports position, velocity, and effort command interfaces and the
corresponding state interfaces. Temperature and fault state interfaces are
optional. The component also handles lifecycle activation, startup readback,
feedback timeouts, motor watchdog configuration, Run-mode recovery, command
clamping, and repeated stop commands during shutdown.

CAN traffic is exchanged through configurable `can_msgs/msg/Frame` topics, so
the component is not tied to a particular SocketCAN bridge. See the
[repository README](https://github.com/s2015-turtle/robstride_ros2#readme) for
the complete hardware and joint parameter reference, controller examples, and
operational limits.
