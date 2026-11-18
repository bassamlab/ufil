^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_central_fusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* The node is a composable component (``ufil_central_fusion::CentralFusionNode``); the
  ``central_fusion_node`` executable is generated from it. Node name changed from
  ``central_fusion_node`` to ``central_fusion``; the hard-coded ``/rsu`` namespace was removed.
* Measurement buffering, out-of-order replay and time-jump handling moved from the node into
  the ROS-independent ``CentralFusion`` class. The tracker internals are private.
* Sensors are configured by name (``sensor_names``, ``<name>.topic``, ``<name>.is_cam``,
  ``<name>.trust.*``, optional ``<name>.fov_topic`` / ``<name>.occlusion_topic``) instead of
  positional string arrays. Fusion parameters are grouped (``fusion.*``, ``association.*``,
  ``existence.*``, ``dimension.*``) and typed; ``timer_period_ms`` became ``fusion.period`` [s],
  ``buffer_time`` became ``fusion.buffer_duration``. Added ``target_frame`` and
  ``output.min_existence_probability`` (both were hard-coded).
* Fixed: the default configuration could not be loaded (string values for double parameters,
  sensor arrays of the wrong length).
* Fixed: the first message after a time jump was dropped.
* Added ``launch/default.launch.xml``, diagnostics and unit tests. Removed the unused OpenCV
  dependency.
* Invalid parameters are logged as fatal before the construction of the component fails.
* The transition model aliases are no longer part of the public ``definitions.hpp``.
