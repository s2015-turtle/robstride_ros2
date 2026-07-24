^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robstride_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add a ros2_control SystemInterface adapter backed by ``robstride_driver``.
* Support position, velocity, and effort command-mode switching per joint.
* Reject non-finite joint values, invalid watchdog values, and blank CAN topic
  names during hardware configuration.
* Contributors: Yamato.K
