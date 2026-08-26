^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robstride_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2026-08-26)
------------------
* Clarify that ``gear_ratio`` applies an ideal additional transmission to
  position, velocity, and effort.
* Contributors: Yamato.K

0.1.1 (2026-08-25)
------------------
* Use ``forward_command_controller`` for position, velocity, and effort
  examples across all supported ROS distributions.
* Contributors: Yamato.K

0.1.0 (2026-08-09)
------------------
* Add launch, controller configuration, and RS/EduLite actuator profiles.
* Expose optional position, velocity, and effort command limits in every motor
  profile macro.
* Require explicit ``kp`` and ``kd`` arguments for every motor-profile macro.
* Contributors: Yamato.K
