^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Repository changelog for robstride_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
