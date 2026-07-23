# robstride_ros2

An unofficial, community-maintained ROS 2 `ros2_control` hardware component for
RobStride actuators using the RobStride private CAN protocol.
CAN frames are exchanged through `can_msgs/msg/Frame` topics. The core packages
depend on `can_msgs` and do not require a specific SocketCAN bridge.

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
- Motor feedback for position, velocity, torque, temperature, and faults
- Startup parameter readback and motor-enable confirmation
- Feedback timeout handling and repeated stop commands during shutdown
- A nonzero motor-side CAN watchdog configured on every activation

## Package structure

The repository contains four ROS 2 packages:

| Package | Responsibility |
|---|---|
| `robstride_driver` | Private protocol and `can_msgs` topic transport |
| `robstride_ros2_control` | `ros2_control` Hardware Component |
| `robstride_examples` | Motor-profile Xacro, controller configuration, and example launch |
| `robstride_ros2` | Installs the complete set of packages |

The plugin identifier `robstride_ros2/RobStrideSystem` remains compatible with
existing robot descriptions.

## CAN bridge compatibility

`robstride_driver` and `robstride_ros2_control` communicate only through
`can_msgs/msg/Frame` topics. They can be used with any CAN bridge that provides
compatible transmit and receive topics.

`robstride_examples` is the only package that directly depends on
[`ros2_socketcan`](https://github.com/autowarefoundation/ros2_socketcan), because
its example launch starts that bridge. The aggregate `robstride_ros2` package
installs the examples as well. To install only the bridge-independent Hardware
Component and driver, build through `robstride_ros2_control`:

```bash
colcon build --packages-up-to robstride_ros2_control --symlink-install
```

## Supported ROS 2 distributions

The same source branch targets all currently supported ROS 2 distributions.
CI is configured to build and test every row independently with its Tier 1
Ubuntu platform.

| ROS 2 distribution | Ubuntu |
|---|---|
| Humble | 22.04 (Jammy) |
| Jazzy | 24.04 (Noble) |
| Kilted | 24.04 (Noble) |
| Lyrical | 26.04 (Resolute) |
| Rolling | 26.04 (Resolute) |

No distribution-specific source branch is required.

Rolling follows current upstream development and can introduce incompatible
API changes before the next stable ROS 2 release. The Rolling CI job is the
compatibility signal for the exact source revision being tested.

## Protocol and firmware assumptions

This package uses the 29-bit extended-frame private protocol documented in the
official RobStride manuals. The motor must therefore be configured for the
private protocol, not CANopen or the separate 11-bit MIT protocol. The listed
EL05 and RS-series profiles use the common command and feedback formats defined
in those manuals.

The profile limits below are the ranges used to encode and decode CAN values,
not recommended mechanical operating limits. Apply tighter joint, velocity, and
effort limits in your robot description when required.

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
| `can_tx_topic` | `to_can_bus` | Outgoing `can_msgs/msg/Frame` topic |
| `can_rx_topic` | `from_can_bus` | Incoming `can_msgs/msg/Frame` topic |
| `can_rx_qos_depth` | `32` | Reliable, volatile feedback QoS depth; increase for large motor groups |
| `feedback_timeout_ms` | `3000` | Maximum time without motor feedback before returning ERROR |
| `fail_on_feedback_timeout` | `true` | Stop the hardware when feedback times out |
| `run_mode_recovery_timeout_ms` | `500` | Time allowed for an active motor to recover to Run mode before returning ERROR |
| `run_mode_recovery_retry_interval_ms` | `100` | Minimum interval between automatic enable retries |
| `clear_faults_on_activate` | `true` | Clear motor faults during activation |
| `set_zero_on_activate` | `false` | Set the current motor position as mechanical zero |
| `shutdown_stop_repetitions` | `3` | Number of zero and stop commands sent at shutdown |
| `shutdown_stop_interval_ms` | `20` | Delay between repeated shutdown frames |
| `shutdown_confirmation_timeout_ms` | `300` | Wait for Reset mode feedback; `0` disables confirmation |
| `startup_connection_timeout_ms` | `3000` | Wait for the CAN bridge topic endpoints |
| `startup_confirmation_timeout_ms` | `500` | Per-attempt parameter and enable confirmation timeout |
| `startup_retries` | `3` | Number of startup parameter and enable attempts |

While active, the hardware monitors each motor's operating state. If a motor
unexpectedly leaves Run mode, the component logs a warning and attempts to
enable it again. Commands resume after Run mode is confirmed. If recovery does
not complete within `run_mode_recovery_timeout_ms`, the hardware reports an
error and stops all motors.

Minimal example using the default settings:

```xml
<hardware>
  <plugin>robstride_ros2/RobStrideSystem</plugin>
</hardware>
```

## Joint parameters

Each `<joint>` requires its own motor settings.

| Parameter | Default | Description |
|---|---:|---|
| `can_id` | required | Unique motor CAN ID in the range `1..255` |
| `can_timeout_ticks` | required | Nonzero motor-side CAN watchdog; 20,000 ticks equals 1 second |
| `position_min/max` | required | CAN encoding range in radians |
| `velocity_min/max` | required | CAN encoding range in rad/s |
| `effort_min/max` | required | ROS effort-command clamp in Nm |
| `effort_wire_min/max` | effort limits | CAN encoding range in Nm |
| `kp_max` / `kd_max` | required | Gain encoding limits |
| `kp` / `kd` | required | Gains used for position and velocity command interfaces |
| `direction` | `1` | Joint direction; either `1` or `-1` |
| `gear_ratio` | `1.0` | Additional protocol-side rotations per ROS joint rotation |
| `position_offset` | `0.0` | ROS joint position offset in radians |
| `command_position_min/max` | full position range | Operational position-command limits in ROS joint radians |
| `command_velocity_min/max` | full velocity range | Operational velocity-command limits in ROS joint rad/s |
| `command_effort_min/max` | `effort_min/max` | Operational effort-command limits in ROS joint Nm |

`gear_ratio` is an additional transmission transform for the surrounding robot.
It is not the actuator's built-in reduction ratio. Leave it at `1.0` when the
private-protocol angle already represents the actuator output used as the ROS
joint position.

The optional `command_*` limits let a robot use a narrower operating envelope
without changing the motor model's CAN encoding ranges. They are expressed in
ROS joint coordinates. Configuration fails if a limit is non-finite, reversed,
or falls outside the corresponding CAN range after applying `direction`,
`gear_ratio`, and `position_offset`.

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
    can_id="1" direction="1" kp="20.0" kd="0.8"
    command_velocity_min="-5.0" command_velocity_max="5.0"
    command_effort_min="-2.0" command_effort_max="2.0"/>
  <xacro:robstride_joint_interfaces/>
</joint>
```

The common `4000` watchdog value is a package safety default, not a factory default.
The official manuals document `0` as the factory value, which disables the
watchdog, and `20000` ticks as 1 second. This package rejects zero so that loss
of host commands eventually forces the motor back to Reset mode.

## Controllers and command modes

| Command interface | Controller type | Unit | Motor command behavior |
|---|---|---|---|
| position | `position_controllers/JointGroupPositionController` | rad | position, Kp, and Kd |
| velocity | `velocity_controllers/JointGroupVelocityController` | rad/s | velocity and Kd; Kp is zero |
| effort | `effort_controllers/JointGroupEffortController` | Nm | torque feed-forward; Kp and Kd are zero |

The hardware maps the active ROS command interface to the corresponding motor
command. A joint cannot claim more than one command interface at a time, but
separate joints may use different modes simultaneously.

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
| `feedback_timeout_ms` | The hardware returns ERROR after motor feedback stops |
| `shutdown_stop_repetitions` | Zero commands and stop commands are repeated |
| `shutdown_confirmation_timeout_ms` | The hardware waits for Reset mode feedback |

On deactivation, shutdown, error, or destruction while active, the component
sends a zero command followed by a stop command to every motor. If the ROS
transport cannot deliver those commands, the configured motor-side CAN
watchdog is the final fallback.

## License

[MIT](LICENSE)

## References

- [RobStride Product Information, revision `6ad12f5` (July 14, 2026)](https://github.com/RobStride/Product_Information/tree/6ad12f50006273b7ea4eea88980f927d97c22f0d)
- [`ros2_socketcan`](https://github.com/autowarefoundation/ros2_socketcan)
