^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_osc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The node is a composable component (``ufil_osc::OscMessageConverterNode``); the
  ``osc_message_converter_node`` executable is generated from it. The default node name changed
  from ``osc_publisher_node`` to ``osc_message_converter``.
* Sending moved into the ROS-independent ``ufil_osc::OscSender`` library, which reports counters
  and errors to the component.
* Parameters renamed and grouped: ``host`` and ``port`` became ``receiver.host`` and
  ``receiver.port``. ``osc_address`` and ``message_type`` are unchanged.
* The ``source_topic`` parameter was replaced by the ``input`` topic, which is remapped by the
  launch files. Both message types are subscribed with the sensor data QoS (best effort); object
  lists used a reliable subscription before.
* Fixed: ``launch/osc.launch.xml`` set parameters the node did not declare (``osc_host``,
  ``osc_port``, ...), so all arguments except ``osc_address`` and ``osc_use_sim_time`` were
  ignored. It is now a wrapper around the new ``launch/default.launch.xml`` with the same
  arguments; the unused ``osc_enable_stats`` argument was removed.
* Fixed: messages whose OSC packet exceeds one UDP datagram are dropped and reported instead of
  failing with an exception from a fixed 64 KiB stack buffer. Socket errors are detected and
  reported.
* Replaced oscpack by a small OSC 1.0 encoder: the oscpack 1.1.0 of Ubuntu aborts on 64 bit
  non-x86 platforms (``osc::int32`` is 8 bytes there). The package has no dependency on
  ``liboscpack-dev`` anymore and ``cmake/FindOSCPACK.cmake`` was removed.
* Invalid parameters or an unresolvable receiver are logged as fatal before the construction of
  the component fails, instead of calling ``rclcpp::shutdown()``.
* Sent messages are no longer logged at INFO. Added diagnostics (sent / too large / failed
  messages, last error), ``config/default.yaml`` and unit tests.
