^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_etsi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The nodes are composable components (``ufil_etsi::VehicleCamGeneratorNode``,
  ``ufil_etsi::VehicleCamReceiverNode``, ``ufil_etsi::RsuCamGeneratorNode``); the executables
  ``vehicle_cam_generator_node``, ``vehicle_cam_receiver_node`` and ``rsu_cam_generator_node``
  are generated from them. Node names, topics and the launch files
  ``vehicle_cam_generation.launch.xml`` / ``vehicle_cam_reception.launch.xml`` are unchanged,
  except for the removed ``*_enable_stats`` arguments (the parameter was never declared).
* New ROS-independent core library: the ETSI encoding and decoding of the CAM data elements, the
  ITS time conversion, the CAM generation rules and the path history no longer depend on ROS
  messages. The ``CamBuilder`` / ``*ContainerBuilder`` classes and ``Utilities.hpp`` were
  replaced by it.
* Added ``launch/vehicle_cam_generator.launch.xml``, ``launch/vehicle_cam_receiver.launch.xml``,
  ``launch/rsu_cam_generator.launch.xml`` and their ``config/*.yaml``; all parameters are
  declared, validated and read-only. Added diagnostics for the vehicle CAM generator and
  receiver.
* RSU CAM generator: the ``position`` array became ``position.latitude``,
  ``position.longitude`` and ``position.altitude`` (default: the CPM lab); added
  ``station_id`` (-1: random as before) and ``time_shift_nanosec``.
* Fixed (generator): the path history was always relative to the first CAM position and
  exceeded the ETSI value range while driving; it now holds the positions of the previous CAMs,
  each relative to the preceding point, with unavailable altitude. The lateral acceleration was
  computed but never marked present. Heading changes across north triggered CAMs. Confidences
  overflowed their 8 bit range instead of being "out of range", yaw rate confidences below
  1 deg/s were reported as 0.01 deg/s, speeds and yaw rates above the range were reported as
  "unavailable", the drive direction was wrong for left-hand angular differences, the first CAM
  had no curvature/vehicle role/exterior lights. CAMs triggered by dynamics now respect
  T_GenCamMin (100 ms), T_GenCam is limited to [T_GenCamMin, T_GenCamMax], every object is used
  for at most one CAM and the generation follows ROS time (``use_sim_time``).
* Fixed (generator): the confidence ellipse is the principal axes ellipse of the position
  covariance instead of always pointing north (also when east was the larger axis). Unknown
  quantities (negative variance) are encoded as unavailable.
* Fixed (receiver): trucks were classified as motorcycles and road side units as trucks
  (comparison with or-ed station types). The ``output_topic`` remap of
  ``vehicle_cam_reception.launch.xml`` had no effect. A CAM without position crashed the node.
  Rotated confidence ellipses were decoded with a wrong rotation. The generation delta time is
  resolved to the closest time, also across its wrap-around. The middle of the yaw rate
  confidence classes 1 deg/s and 5 deg/s is used as intended.
* Timestamps before 2004 (missing ``time_shift_nanosec``) are reported instead of throwing.
* Removed the unused dependencies (eigen, nav_msgs, sensor_msgs, geometry_msgs,
  tf2_geometry_msgs, etsi_its_msgs_utils) and the lint skips; added unit tests.
