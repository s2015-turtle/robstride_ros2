^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robstride_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-11-04)
------------------
* Synchronize all four packages at 0.3.0 for strict operational command rejection,
  activation checks, transport robustness and current Rolling compatibility.
* Preserve the aggregate installation entry point and plugin identifier
  ``robstride_ros2/RobStrideSystem``.
* Include controller_manager-to-vcan integration coverage on Jazzy and Rolling.
* Contributors: Yamato.K

0.2.0 (2026-09-06)
------------------
* Synchronize all packages at 0.2.0 for diagnostics, transport health,
  production-owned motor profiles, and virtual CAN integration coverage.
* Preserve the aggregate installation entry point and existing plugin name.
* Contributors: Yamato.K

0.1.2 (2026-08-26)
------------------
* Synchronize the aggregate package version for the effort transmission
  conversion fix.
* Contributors: Yamato.K

0.1.1 (2026-08-25)
------------------
* Update the example-controller dependency set for all supported ROS
  distributions.
* Contributors: Yamato.K

0.1.0 (2026-08-09)
------------------
* Keep ``robstride_ros2`` as the aggregate installation entry point while the
  implementation is split into driver, ros2_control, and example packages.
* Move development headers to ``robstride_ros2_control`` and example resources
  to ``robstride_examples`` while preserving the existing plugin identifier.
* Contributors: Yamato.K
