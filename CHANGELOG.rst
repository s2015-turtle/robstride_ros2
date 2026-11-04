^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Repository changelog for robstride_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-11-04)
------------------
* Reject out-of-range operational commands instead of clamping. An invalid
  command withholds the complete multi-joint command batch and reports an error;
  correct the command and reactivate the hardware/controller before resuming.
* Require fresh, finite feedback within operational limits before activating
  position control, and preserve neutral velocity and effort commands.
* Strictly validate numeric parameters and handle position-transform roundoff
  at motor endpoints without widening operational limits.
* Support framework-managed interfaces in ros2_control 6.12 and later while
  retaining compatibility with older interfaces and Humble multi-hardware switching.
* Contain CAN executor exceptions and prevent lost shutdown wakeups.
* Extend fake-motor tests with watchdog expiry and independent multi-axis faults.
* Test the installed plugin through controller_manager, ros2_socketcan and vcan
  on Jazzy and Rolling, including commands, feedback, switching and reactivation.
* The example now sets mechanical zero on activation and uses a 0.05-second CAN
  receiver interval. The driver's default for ``set_zero_on_activate`` remains
  false; review the example setting before using an existing calibrated robot.
* Preserve the existing automatic Run-mode recovery policy. Issue #40 remains
  under design review; the explicit-reactivation policy in draft PR #47 is not included.
* A publish call that never returns can still prevent shutdown completion
  (issue #31). These changes do not establish a bounded shutdown guarantee or
  physical safety certification.
* Contributors: Yamato.K, ryo-0723

0.2.0 (2026-09-06)
------------------
* Add human-readable motor diagnostics, CAN traffic metrics and persistent
  transmit-failure propagation.
* Centralize motor profiles in the production driver and reduce contention
  in control updates.
* Add driver and transport integration tests with a fake motor over vcan.
* Contributors: Yamato.K

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
