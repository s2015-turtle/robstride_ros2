^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Repository changelog for robstride_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2026-08-26)
------------------
* Apply ``gear_ratio`` to effort commands and feedback using the same ideal
  transmission convention as position and velocity.
* Derive and validate ROS joint effort limits in joint coordinates while
  keeping motor clamp and CAN encoding limits in motor coordinates.
* Document the motor-side and joint-side effort parameter semantics.
* Contributors: Yamato.K

0.1.1 (2026-08-25)
------------------
* Use ``forward_command_controller`` for position, velocity, and effort
  examples across all supported ROS 2 distributions.
* Replace the distribution-specific example controller dependencies with the
  common ``forward_command_controller`` dependency.
* Keep all four packages on the same release version.
* Contributors: Yamato.K

0.1.0 (2026-08-09)
------------------
* Add a ros2_control SystemInterface for the RobStride private CAN protocol.
* Support position, velocity, and effort control for multiple actuators.
* Add RS00 through RS06 and EduLite EL05 model profiles.
* Add startup confirmation, feedback timeout handling, repeated shutdown
  commands, and a motor-side CAN watchdog.
* Add compatibility with ROS 2 Humble, Jazzy, Kilted, Lyrical, and Rolling.
* Use ``can_msgs/msg/Frame`` topics for CAN transport; the example launch uses
  ``ros2_socketcan`` as its bridge.
* Move periodic motion-frame publication to a latest-value transport thread so
  the controller update loop cannot accumulate stale commands before DDS publication.
* Serialize all outbound frames through one DDS DataWriter and invalidate
  frames from earlier hardware activations with a generation counter.
* Keep one pending motion frame per motor before DDS publication and expose
  only the receive QoS depth as a hardware parameter.
* Separate SocketCAN topic transport from the ros2_control hardware lifecycle,
  and group joint runtime data by state, command, claim, and response status.
* Reduce ``RobStrideSystem`` to a thin ros2_control adapter and move configuration,
  joint runtime state, motor operations, and recovery into internal driver modules.
* Monitor Type 2 mode while active, retry enable through the transport worker,
  and escalate persistent recovery failures through the hardware ERROR path.
* Split the repository into driver, ros2_control, examples, and compatibility
  packages with a one-way dependency graph.
* Contributors: Yamato.K
