^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ssl_wheel_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-11-02)
------------------
* New C++ package replacing the Python packages ``ssl_wheel_detection`` (0.1.0) and
  ``ssl_wheel_tracking`` (0.1.0). One composable component
  (``ssl_wheel_tracker::SslWheelTrackerNode``, executable ``ssl_wheel_tracker_node``) subscribes
  to the pressure grids of all mats (``input_topics``) and publishes ``wheel_tracks``; the
  intermediate topic is no longer needed. The detected wheels are still published as
  ``wheel_detections`` for debugging.
* Detection and tracking live in the ROS-independent ``WheelDetector`` and ``WheelTracker``
  classes. The Python dependencies scipy, scikit-image and Stone Soup are removed.
* Detection reimplements the former ``peak_local_max`` / nearest-neighbour / dilation pipeline.
  Changed: the detected pressure (``Wheel/intensity``) is the pressure sum above the background
  and no longer scaled by the grid resolution; detections are published in ``tracking_frame``.
* Tracking reimplements the former Stone Soup tracker (constant velocity Kalman filter, global
  nearest neighbour assignment with a 2 m missed distance, confirmation after 4 detections,
  deletion after 0.25 s without detection). Changed:

  * Track ids are sequential numbers assigned on confirmation instead of UUIDs.
  * A track publishes its last ``history_length`` matched states and the predictions since the
    last match; the former history grew without bound and skipped the first state.
  * Frames that are not newer than the last tracked frame are dropped instead of predicting
    backwards; a backward time jump larger than ``deletion_timeout`` resets the tracker.
  * Transforms are looked up without blocking; grids without transform are dropped.

* The tracking frame (former hard-coded ``map``), the detection and tracking parameters and the
  update period (former 10 ms) are declared, validated, read-only parameters. The unused
  ``enable_stats`` parameter of the former launch files is removed.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
