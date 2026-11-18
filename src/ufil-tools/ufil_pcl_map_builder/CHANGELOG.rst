^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_pcl_map_builder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* The ROS-independent ``PointMap`` holds the map and the Lanelet2 OSM export; the node
  (``PclMapBuilderNode``, name ``pcl_map_builder``) only converts messages and parameters.
* Parameters renamed: ``reference_lat``/``reference_lon`` to ``origin.latitude``/
  ``origin.longitude``. Added ``point_types`` (was hard-coded) and ``point_type``, which can be
  changed at runtime with ``ros2 param set``. Parameters are validated at startup.
* The keyboard type switcher only runs in an interactive terminal, so the node no longer
  crashes under ``ros2 launch``. The terminal is restored on shutdown.
* Without ``autosave`` the map is written on shutdown instead of never.
* The subscription is relative (``clicked_point``); the launch file remaps it to
  ``/clicked_point``.
* Added ``config/default.yaml``, ``launch/default.launch.xml`` and tests.
