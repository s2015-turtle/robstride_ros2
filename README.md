# robstride_ros2

An unofficial, community-maintained ROS 2 `ros2_control` hardware component for
RobStride actuators using the RobStride private CAN protocol.
CAN frames are exchanged through the `can_msgs/msg/Frame` topics provided by
[`ros2_socketcan`](https://github.com/autowarefoundation/ros2_socketcan).

This project is not affiliated with or endorsed by RobStride. The protocol and
model profiles were checked against the English manuals in RobStride's official
[`Product_Information`](https://github.com/RobStride/Product_Information)
repository. A Japanese README is available as
[`README.ja.md`](README.ja.md).

## Features

- One `ros2_control` SystemInterface for multiple actuators on one CAN bus
- Position, velocity, and effort command interfaces for every joint
- Different command modes for different motors at the same time
- Model profiles for RS00, RS01, RS02, RS03, RS04, RS05, RS06, and EL05
- Type 2 feedback for position, velocity, torque, temperature, and faults
- Startup parameter readback and motor-enable confirmation
- Feedback timeout handling and repeated Type 4 stop frames during shutdown
- A nonzero motor-side CAN watchdog configured on every activation

## Package structure

The repository is a four-package workspace with a one-way dependency graph:

| Package | Responsibility |
|---|---|
| `robstride_driver` | Public driver library: protocol, `ros2_socketcan` topic transport, motor lifecycle, feedback, and Run-mode recovery |
| `robstride_ros2_control` | Thin `hardware_interface::SystemInterface` adapter and plugin registration |
| `robstride_examples` | Motor-profile Xacro, controller configuration, and example launch |
| `robstride_ros2` | Compatibility entry point preserving the original launch, configuration, Xacro, and plugin identifiers |

`robstride_driver` does not depend on `hardware_interface`. HardwareInfo parsing
and command-interface switching belong to `robstride_ros2_control`, which maps
the standard lifecycle callbacks to the driver's `open`, `start`,
`update_state`, and `send_commands` operations.
Installable headers live under each package's `include/` tree. Private adapter
headers live under `robstride_ros2_control/internal/` and are never installed.
The plugin, launch, and Xacro identifiers remain compatible. Code that directly
included the former `<robstride_ros2/robstride_system.hpp>` development header
must migrate to `<robstride_ros2_control/robstride_system.hpp>` and link
`robstride_ros2_control`.

## Supported ROS 2 distributions

The same source branch targets all currently supported ROS 2 distributions.
CI is configured to build and test every row independently with its Tier 1
Ubuntu platform.

| ROS 2 distribution | Ubuntu | `ros2_control` API |
|---|---|---|
| Humble | 22.04 (Jammy) | 2.x |
| Jazzy | 24.04 (Noble) | 4.x |
| Kilted | 24.04 (Noble) | 5.x |
| Lyrical | 26.04 (Resolute) | 6.x |
| Rolling | 26.04 (Resolute) | current development version |

Humble and Jazzy use the original `HardwareInfo` initialization callback.
Kilted, Lyrical, and Rolling use the newer
`HardwareComponentInterfaceParams` callback selected at build time. No
distribution-specific source branch is required.

Rolling follows current upstream development and can introduce incompatible
API changes before the next stable ROS 2 release. The Rolling CI job is the
compatibility signal for the exact source revision being tested.

## Protocol and firmware assumptions

This package uses the 29-bit extended-frame private protocol described by the
official manuals dated July 13, 2026. The motor must therefore be configured for
the private protocol, not CANopen or the separate 11-bit MIT protocol.

The current official EL05 and RS-series manuals specify the same Type 1 command
and Type 2 feedback layouts. In particular, Type 2 contains a 16-bit normalized
angle, velocity, torque, and temperature. The manuals do not describe a
model-specific Type 2 packet or explicitly state a single-encoder versus
dual-encoder packet distinction. Encoder topology may affect how firmware
obtains the reported angle; this package therefore applies the documented
common Type 2 layout to all listed profiles.

The profile limits below are protocol normalization ranges, not recommended
mechanical operating limits. Apply tighter joint, velocity, and effort limits in
your robot description when required.

| Model | Position | Velocity | Torque | Kp | Kd |
|---|---:|---:|---:|---:|---:|
| RS00 | +/-4 pi rad | +/-33 rad/s | +/-14 Nm | 0..500 | 0..5 |
| RS01 | +/-4 pi rad | +/-44 rad/s | +/-17 Nm | 0..500 | 0..5 |
| RS02 | +/-4 pi rad | +/-44 rad/s | +/-17 Nm | 0..500 | 0..5 |
| RS03 | +/-4 pi rad | +/-20 rad/s | +/-60 Nm | 0..5000 | 0..100 |
| RS04 | +/-4 pi rad | +/-15 rad/s | +/-120 Nm | 0..5000 | 0..100 |
| RS05 | +/-4 pi rad | +/-50 rad/s | +/-5.5 Nm | 0..500 | 0..5 |
| RS06 | +/-4 pi rad | +/-50 rad/s | +/-36 Nm | 0..5000 | 0..100 |
| EL05 | +/-4 pi rad | +/-50 rad/s | +/-6 Nm | 0..500 | 0..5 |

## Installation

Install the package in the `src` directory of a workspace for one of the
supported distributions, then resolve dependencies and build it:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to robstride_ros2 --symlink-install
source install/setup.bash
```

Bring up the SocketCAN interface before launching the example. RobStride's
private protocol uses 1 Mbit/s CAN and extended frames:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

## Example launch

The example uses motor CAN ID 1 with the EL05 profile and starts the position
controller. The canonical package is `robstride_examples`:

```bash
ros2 launch robstride_examples robstride_example.launch.py interface:=can0
```

The original command remains available through the compatibility package:

```bash
ros2 launch robstride_ros2 robstride_example.launch.py interface:=can0
```

Inspect the loaded hardware and feedback:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

Send a position command in radians:

```bash
ros2 topic pub --rate 20 \
  /robstride_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2]}"
```

## Hardware parameters

Place these parameters inside the `<hardware>` element of the `ros2_control`
description.

| Parameter | Default | Description |
|---|---:|---|
| `host_can_id` | `253` | Host CAN ID in the range `0..255` |
| `can_tx_topic` | `to_can_bus` | Frames sent to `ros2_socketcan` |
| `can_rx_topic` | `from_can_bus` | Frames received from `ros2_socketcan` |
| `can_rx_qos_depth` | `32` | Reliable, volatile feedback QoS depth; increase for large motor groups |
| `feedback_timeout_ms` | `3000` | Maximum time without Type 2 feedback before returning ERROR |
| `fail_on_feedback_timeout` | `true` | Stop the hardware when feedback times out |
| `run_mode_recovery_timeout_ms` | `500` | Time allowed for an active motor to recover to Run mode before returning ERROR |
| `run_mode_recovery_retry_interval_ms` | `100` | Minimum interval between automatic Type 3 enable retries |
| `clear_faults_on_activate` | `true` | Send a Type 4 fault-clear request during activation |
| `set_zero_on_activate` | `false` | Set the current motor position as mechanical zero |
| `shutdown_stop_repetitions` | `3` | Number of zero commands and Type 4 stop frames sent at shutdown |
| `shutdown_stop_interval_ms` | `20` | Delay between repeated shutdown frames |
| `shutdown_confirmation_timeout_ms` | `300` | Wait for Type 2 Reset mode; `0` disables confirmation |
| `startup_connection_timeout_ms` | `3000` | Wait for the `ros2_socketcan` topic endpoints |
| `startup_confirmation_timeout_ms` | `500` | Per-attempt parameter and enable confirmation timeout |
| `startup_retries` | `3` | Number of startup parameter and enable attempts |

The deprecated `can_qos_depth` name is accepted as an alias for
`can_rx_qos_depth`. New descriptions should use the current name.

All outgoing CAN frames pass through one transport worker and one DDS
DataWriter, so lifecycle transactions and active commands have a single publish
order. Before publication, each motor has one pending Type 1 slot; a newer
command replaces an older unsent command. Transactions remain FIFO. Once a
sample has been handed to DDS, delivery and buffering are governed by the RMW
and `ros2_socketcan`; the driver does not claim per-motor latest-value semantics
inside those downstream queues.

While the hardware lifecycle state is active, every Type 2 response is also
checked for Run mode. If a motor unexpectedly reports Reset or another non-Run
mode, the component logs a warning and retries Type 3 enable through the
transport worker, without synchronous DDS publication in the update loop. A
motor's Type 1 slot is held during recovery and released only after a subsequent
Run response. If Run is not confirmed before `run_mode_recovery_timeout_ms`, `read()` returns
ERROR; controller manager then invokes the hardware error lifecycle handling,
which disables all motors and stops the transport.

Deactivation increments an active-generation counter, discards pending active
frames, and waits for any publication already in progress before queuing stop
transactions. Frames extracted by the worker under an older generation are
rejected even if the hardware is later reactivated. Transport shutdown drains
queued lifecycle transactions before stopping the worker, so locally queued
zero-command and Type 4 frames are not discarded.

Example:

```xml
<hardware>
  <plugin>robstride_ros2/RobStrideSystem</plugin>
  <param name="host_can_id">253</param>
  <param name="can_tx_topic">to_can_bus</param>
  <param name="can_rx_topic">from_can_bus</param>
  <param name="can_rx_qos_depth">32</param>
  <param name="feedback_timeout_ms">3000</param>
  <param name="fail_on_feedback_timeout">true</param>
  <param name="run_mode_recovery_timeout_ms">500</param>
  <param name="run_mode_recovery_retry_interval_ms">100</param>
  <param name="clear_faults_on_activate">true</param>
  <param name="set_zero_on_activate">false</param>
  <param name="shutdown_stop_repetitions">3</param>
  <param name="shutdown_stop_interval_ms">20</param>
  <param name="shutdown_confirmation_timeout_ms">300</param>
  <param name="startup_connection_timeout_ms">3000</param>
  <param name="startup_confirmation_timeout_ms">500</param>
  <param name="startup_retries">3</param>
</hardware>
```

## Joint parameters

Each `<joint>` requires its own motor settings.

| Parameter | Default | Description |
|---|---:|---|
| `can_id` | required | Unique motor CAN ID in the range `1..255` |
| `can_timeout_ticks` | required | Nonzero motor-side CAN watchdog; 20,000 ticks equals 1 second |
| `position_min/max` | required | Type 1/2 wire normalization range in radians |
| `velocity_min/max` | required | Type 1/2 wire normalization range in rad/s |
| `effort_min/max` | required | ROS effort-command clamp in Nm |
| `effort_wire_min/max` | effort limits | Type 1/2 wire normalization range in Nm |
| `kp_max` / `kd_max` | required | Type 1 gain normalization limits |
| `kp` / `kd` | required | Gains used for position and velocity command interfaces |
| `direction` | `1` | Joint direction; either `1` or `-1` |
| `gear_ratio` | `1.0` | Additional protocol-side rotations per ROS joint rotation |
| `position_offset` | `0.0` | ROS joint position offset in radians |

`gear_ratio` is an additional transmission transform for the surrounding robot.
It is not the actuator's built-in reduction ratio. Leave it at `1.0` when the
private-protocol angle already represents the actuator output used as the ROS
joint position.

Every joint must export all three command interfaces and the three required
state interfaces. `temperature` and `fault` are optional state interfaces:

```xml
<command_interface name="position"/>
<command_interface name="velocity"/>
<command_interface name="effort"/>

<state_interface name="position"/>
<state_interface name="velocity"/>
<state_interface name="effort"/>
<state_interface name="temperature"/>
<state_interface name="fault"/>
```

## Model profile macros

The predefined macros are in
[`robstride_examples/description/robstride_motor_profiles.xacro`](robstride_examples/description/robstride_motor_profiles.xacro).

| Model | Macro | Default watchdog ticks |
|---|---|---:|
| RS00 | `robstride_rs00_params` | `4000` |
| RS01 | `robstride_rs01_params` | `4000` |
| RS02 | `robstride_rs02_params` | `4000` |
| RS03 | `robstride_rs03_params` | `4000` |
| RS04 | `robstride_rs04_params` | `4000` |
| RS05 | `robstride_rs05_params` | `4000` |
| RS06 | `robstride_rs06_params` | `4000` |
| EL05 | `robstride_edulite05_params` | `4000` |

Example:

```xml
<xacro:include filename="$(find robstride_examples)/description/robstride_motor_profiles.xacro"/>

<joint name="wheel_joint_1">
  <xacro:robstride_edulite05_params
    can_id="1" direction="1" kp="20.0" kd="0.8"/>
  <xacro:robstride_joint_interfaces/>
</joint>
```

The common `4000` watchdog value is a package safety default, not a factory default.
The official manuals document `0` as the factory value, which disables the
watchdog, and `20000` ticks as 1 second. This package rejects zero so that loss
of host commands eventually forces the motor back to Reset mode.

## Controllers and command modes

| Command interface | Controller type | Unit | Type 1 fields |
|---|---|---|---|
| position | `position_controllers/JointGroupPositionController` | rad | position, Kp, and Kd |
| velocity | `velocity_controllers/JointGroupVelocityController` | rad/s | velocity and Kd; Kp is zero |
| effort | `effort_controllers/JointGroupEffortController` | Nm | torque feed-forward; Kp and Kd are zero |

The hardware keeps every motor in private-protocol operation control mode
(`run_mode = 0`) and maps the active ROS command interface into the appropriate
Type 1 fields. A joint cannot claim more than one command interface at a time,
but separate joints may use different modes simultaneously.

Switch one joint group from position to velocity control:

```bash
ros2 run controller_manager spawner robstride_velocity_controller \
  --inactive --controller-manager /controller_manager

ros2 control switch_controllers \
  --deactivate robstride_position_controller \
  --activate robstride_velocity_controller \
  --controller-manager /controller_manager

ros2 topic pub --rate 20 \
  /robstride_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0]}"
```

Use the same procedure with `robstride_effort_controller` for effort commands.

## Multiple motors

Give every motor a unique joint name and CAN ID, then group joints in controller
configuration. For example, IDs 1-4 can use velocity control while IDs 5-6 use
position control:

```yaml
wheel_velocity_controller:
  ros__parameters:
    joints: [joint_1, joint_2, joint_3, joint_4]

steering_position_controller:
  ros__parameters:
    joints: [joint_5, joint_6]
```

```bash
ros2 topic pub --rate 20 \
  /wheel_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0, 1.0, 1.0]}"

ros2 topic pub --rate 20 \
  /steering_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2, -0.2]}"
```

## Timeout and shutdown behavior

| Setting | Behavior |
|---|---|
| `can_timeout_ticks` | The motor returns to Reset mode after host commands stop |
| `feedback_timeout_ms` | The hardware returns ERROR after Type 2 feedback stops |
| `shutdown_stop_repetitions` | Zero Type 1 commands and Type 4 stop frames are repeated |
| `shutdown_confirmation_timeout_ms` | The hardware waits for Type 2 Reset mode feedback |

On deactivation, shutdown, error, or destruction while active, the component
sends a zero Type 1 command followed by a Type 4 stop frame to every motor. If
the ROS transport cannot deliver those frames, the configured motor-side CAN
watchdog is the final fallback.

## License

[MIT](LICENSE)

## References

- [RobStride Product Information, revision `6ad12f5` (July 14, 2026)](https://github.com/RobStride/Product_Information/tree/6ad12f50006273b7ea4eea88980f927d97c22f0d)
- [ros2_control: Writing a Hardware Component](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- [`ros2_socketcan`](https://github.com/autowarefoundation/ros2_socketcan)
