^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ssl_vehicle_tracking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-11-02)
------------------
* New C++ package replacing the Python package ``ssl_vehicle_detection`` (0.1.0). The node is a
  composable component (``ssl_vehicle_tracking::SslVehicleTrackingNode``, executable
  ``ssl_vehicle_tracking_node``); the default node name changed from ``vehicle_detection_node``
  to ``ssl_vehicle_tracking``.
* Template fitting, evaluation, reassembly and detection live in the ROS-independent
  ``VehicleTracker`` and ``VehicleTemplate`` classes and reproduce the former algorithm,
  including its probability models. The Python dependencies numpy, scipy and the unused Stone Soup
  dependency are removed.
* The vehicle templates moved here from ``ssl_vehicle_detection``. ``vehicle_templates`` accepts
  file names, resolved in the ``templates`` directory of this package, or paths; the former
  default ``['']`` failed at startup. Invalid templates (fewer than two wheels, unknown
  classification) are rejected.
* Changed: vehicle ids are sequential numbers starting at 1, assigned to detected vehicles only;
  the former counter also counted rejected candidates.
* Removed the parameter ``history_size``: only the latest state of every wheel track was
  evaluated. Removed the unused per-vehicle covariance and template offset.
* All parameters are read-only and validated; invalid values are logged as fatal and fail the
  construction. Wheel tracks without state or with a non-finite state are ignored.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
