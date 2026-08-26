^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robstride_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2026-08-26)
------------------
* Derive default ROS joint effort limits from motor limits through
  ``gear_ratio`` and ``direction``.
* Validate explicit joint effort limits after converting them back to motor
  clamp and CAN encoding coordinates.
* Contributors: Yamato.K

0.1.1 (2026-08-25)
------------------
* Synchronize the package version for the multi-package compatibility release.
* Contributors: Yamato.K

0.1.0 (2026-08-09)
------------------
* Add a ros2_control SystemInterface adapter backed by ``robstride_driver``.
* Support position, velocity, and effort command-mode switching per joint.
* Parse and validate optional ROS joint-coordinate command limits separately
  from the CAN encoding ranges.
* Reject non-finite joint values, invalid watchdog values, and blank CAN topic
  names during hardware configuration.
* Contributors: Yamato.K
