^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_message_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The node is a composable component (``ufil_message_filter::ObjectListPositionFilterNode``); the
  ``object_list_position_filter_node`` executable is generated from it. The default node name is
  ``object_list_position_filter``; topics are unchanged (``objects`` -> ``objects_filtered``).
* The area of interest moved into the ROS-independent core library ``ufil_message_filter``
  (``PositionFilter``, ``PositionFilterConfig``). The area is validated (at least 3 finite
  vertices, simple polygon) and may be given in either orientation.
* Parameter ``area_of_interest`` was replaced by ``area_of_interest.enabled`` (default
  ``false``: every object is forwarded) and ``area_of_interest.vertices``
  (``[x1, y1, x2, y2, ...]``, only declared if enabled), because a parameter file cannot hold
  an empty array. All parameters are read-only.
* ``object_list_position_filter.launch.xml`` keeps its argument names except for the removed
  ``object_list_position_filter_enable_stats``, which set an undeclared parameter without any
  effect. ``object_list_position_filter_area_of_interest`` is now required; its old default
  (an empty string) made the node fail.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics with object counters
  and unit tests.
* Removed the unused ``object_position_filter_node`` (``ufil_msgs/ObjectStamped`` filter, not
  started by any launch file) and the unused dependencies on ``nav_msgs``, ``sensor_msgs``,
  ``tf2``, ``tf2_ros``, ``tf2_geometry_msgs``, ``message_filters`` and ``geographiclib``.
