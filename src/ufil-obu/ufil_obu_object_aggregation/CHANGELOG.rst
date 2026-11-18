^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_obu_object_aggregation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* The node is a composable component (``ufil_obu_object_aggregation::ObjectAggregationNode``);
  the ``object_aggregation_node`` executable is generated from it. Node name changed from
  ``object_fusion_node`` to ``object_aggregation``.
* The aggregation (UTM conversion, state, covariance, dimension and class) is a ROS-independent
  core library.
* Parameters renamed to ``object.length``, ``object.width``, ``object.classification`` with
  defaults of a passenger car (previously 0, which failed startup). Added ``utm_zone``.
* Fixed: the frame id was always ``utm_32n``, also when the position is in another UTM zone.
  It now names the zone the position is expressed in.
* Fixed: velocity and acceleration were published in the vehicle frame. They are now rotated
  into the global frame like every UFIL object state, with the covariance propagated including
  the yaw uncertainty.
* Fixed: the width variance was written to the length-width cross-covariance.
* GNSS fixes reported as ``STATUS_NO_FIX`` or with invalid coordinates are no longer published.
* Added ``config/default.yaml``, ``launch/default.launch.xml`` (replaces the Python launch
  file), diagnostics and unit tests. Removed the unused Boost and tf2_ros dependencies.
