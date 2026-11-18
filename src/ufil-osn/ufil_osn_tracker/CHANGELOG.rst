^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_osn_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* The node is a composable component (``ufil_osn_tracker::OsnTrackerNode``); the
  ``ufil_osn_tracker_node`` executable is generated from it.
* The core library no longer depends on ROS 2 (``ufil_msgs`` include removed from the public
  header). Pipeline internals are private; the public types live in ``namespace
  ufil_osn_tracker`` instead of the global namespace.
* Time-jump reset and out-of-order rejection moved into the core library
  (``OsnObjectTracker::update`` returns an ``UpdateResult``). A reset keeps the occlusion grid.
* Parameters are grouped and validated: ``association.threshold``,
  ``track.history_duration`` (was ``history_length``), ``clustering.*`` (was ``cluster_*``),
  ``detector.*_covariance_multiplier`` (fixes the ``mulitplier`` typo). Added
  ``min_existence_probability`` (was hard-coded 0.2).
* Topics renamed: input ``cloud`` to ``dynamic_cloud``, output ``tracks`` to ``object_list``.
* Association counts are logged at debug level instead of info on every cloud.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
* Removed the unused OpenCV dependency.
