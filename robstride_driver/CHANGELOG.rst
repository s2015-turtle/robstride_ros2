^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robstride_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-11-04)
------------------
* Reject out-of-range position, velocity and effort commands without clamping;
  discard queued motion commands and fail the complete multi-joint batch.
* Require fresh, in-range position feedback when claiming position control.
* Preserve neutral commands for asymmetric ranges containing zero and handle
  floating-point roundoff at position-transform endpoints.
* Contain executor exceptions, report executor failure through transport health,
  and clean up failed transport runs before restarting.
* Prevent lost worker shutdown wakeups. A permanently blocked publication can
  still prevent shutdown completion; issue #31 remains unresolved.
* Extend fake-motor coverage with watchdog expiry, independent multi-axis faults
  and a test-only peer for controller_manager integration.
* Retain existing automatic Run-mode recovery behavior.
* Contributors: Yamato.K

0.2.0 (2026-09-06)
------------------
* Add CAN traffic metrics and human-readable motor and transport diagnostics.
* Report motor mode, temperature, faults, feedback age, and recovery attempts.
* Reduce transport contention during control updates and propagate persistent
  transmit-path failures.
* Add virtual CAN transport and fake-motor lifecycle integration tests.
* Centralize RS00-RS06 and EL05 protocol ranges in the motor profile registry.
* Contributors: Yamato.K

0.1.2 (2026-08-26)
------------------
* Apply ``gear_ratio`` when converting effort commands and feedback between
  ROS joint and motor coordinates.
* Add tests for unity and non-unity ratios with both joint directions.
* Contributors: Yamato.K

0.1.1 (2026-08-25)
------------------
* Synchronize the package version for the multi-package compatibility release.
* Contributors: Yamato.K

0.1.0 (2026-08-09)
------------------
* Add the RobStride private-CAN protocol, topic transport, motor lifecycle,
  feedback monitoring, and Run-mode recovery library.
* Clamp position, velocity, and effort commands to per-joint operational
  limits before CAN encoding.
* Contributors: Yamato.K
