# controller_manager + vcan integration

This opt-in Linux test loads the installed `robstride_ros2/RobStrideSystem`
plugin in a real `ros2_control_node`. Standard controller-manager services and
forward-command topics exercise position, velocity and effort control through
`ros2_socketcan` and `vcan0`.

The test-only `vcan_fake_motor_peer` reuses `robstride_driver/test/support`.
Its stdin channel can inject a measurement or request observations, but cannot
inject motor commands. Assertions inspect commands received on the CAN socket,
and independently injected feedback must appear in `/joint_states` by joint name.
The fixture uses nontrivial direction, gear ratio and offset transformations.

Coverage includes initial neutral commands, measured-position initialization,
controller switching, hardware deactivation, CAN stop confirmation and hardware
reactivation with a changed measured position. It does not model mechanics or
establish physical safety or firmware conformance.

Run in an isolated ROS domain with no other controllers or motor publishers.
The runner refuses interfaces whose link type is not `vcan`.

```bash
# On a Linux ROS test machine, from the workspace root:
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
colcon build --base-paths src --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ROS_DOMAIN_ID=86 python3 \
  src/robstride_ros2/robstride_examples/test/integration/test_controller_manager_vcan.py \
  --fake-motor build/robstride_driver/vcan_fake_motor_peer \
  --output-dir /tmp/controller_manager_vcan
```

Ordinary `colcon test` does not run this privileged integration scenario.
GitHub Actions runs it on Jazzy (legacy interfaces) and Rolling (managed
interfaces). Process logs and the fake motor observation transcript are
uploaded as `vcan-logs-<distro>`, including when assertions fail. Service,
discovery and subprocess waits have deadlines; child process groups are stopped
in reverse startup order, with termination escalation if needed.
