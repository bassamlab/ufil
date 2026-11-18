^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples_pedestrian_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The node is a composable component (``ufil_examples_pedestrian_tracker::PedestrianTrackerNode``);
  the ``ufil_examples_pedestrian_tracker_node`` executable is generated from it. Node name
  changed from ``pedestrian_tracker_node`` to ``pedestrian_tracker``.
* The tracker runs on the stamps of the received object lists instead of their reception time,
  so replayed or delayed data is tracked correctly.
* Added the read-only parameters ``process_noise``, ``track_timeout`` and ``max_time_gap`` (were
  hard-coded with the same values).
* Time-jump detection and reset moved from the node into the ROS-independent
  ``PedestrianTracker`` class, which reports it as ``UpdateEvent`` and provides a
  ``TrackerStatus``. The motion and measurement model aliases are no longer public; the public
  header is ``pedestrian_tracker.hpp``.
* Removed the WARN log message on every received object list.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
  ``launch/pedestrian_tracker.launch.xml`` is kept as a wrapper with the former arguments.
* Removed the unused Eigen dependency declaration (Eigen comes with ufil_object_tracking).
