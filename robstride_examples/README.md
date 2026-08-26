# robstride_examples

`robstride_examples` contains the ready-to-run integration files for
`robstride_ros2_control`:

- Xacro motor profiles for RS00 through RS06 and EduLite EL05;
- position, velocity, and effort controller configuration;
- a robot description for a single EL05 actuator;
- a launch file that starts `ros2_socketcan`, `ros2_control`, and the example
  controller.

After bringing up a 1 Mbit/s SocketCAN interface, launch the example with:

```bash
ros2 launch robstride_examples robstride_example.launch.py interface:=can0
```

The default launch uses motor CAN ID 1 and starts the position controller.
Commands are accepted on
`/robstride_position_controller/commands`. Alternative controller definitions
for velocity and effort are included in `config/controllers.yaml`.

The profiles supply protocol encoding ranges, but robot-specific operational
limits and control gains must still be selected for the actual mechanism. See
the [repository README](https://github.com/s2015-turtle/robstride_ros2#readme)
for command examples, profile arguments, multi-motor configuration, and safety
notes.
