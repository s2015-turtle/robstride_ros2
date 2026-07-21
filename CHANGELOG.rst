^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robstride_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add a ros2_control SystemInterface for the RobStride private CAN protocol.
* Support position, velocity, and effort control for multiple actuators.
* Add RS00 through RS06 and EduLite EL05 model profiles.
* Add startup confirmation, feedback timeout handling, repeated shutdown
  commands, and a motor-side CAN watchdog.
* Add compatibility with ROS 2 Humble, Jazzy, Kilted, Lyrical, and Rolling.
* Use ros2_socketcan topics for CAN transport.
* Move periodic motion-frame publication to a latest-value transport thread so
  the controller update loop cannot accumulate stale DDS commands.
* Keep exactly one DDS motion frame per configured motor and expose only the
  receive QoS depth as a hardware parameter.
* Separate SocketCAN topic transport from the ros2_control hardware lifecycle,
  and group joint runtime data by state, command, claim, and response status.
* Monitor Type 2 mode while active, retry enable through the transport worker,
  and escalate persistent recovery failures through the hardware ERROR path.
* Contributors: Yamato.K
