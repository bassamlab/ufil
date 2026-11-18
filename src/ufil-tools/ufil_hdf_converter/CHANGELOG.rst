^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_hdf_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* The tool is the ``ufil_hdf_converter`` command on the ``PATH`` after sourcing the workspace.
  It replaces the ``ros2 run`` executable ``ufil_hdf_converter-node``. No ``rclpy.init`` is
  needed anymore.
* Split into the ROS-independent ``hdf_layout``/``hdf_writer`` modules and the ROS adapter
  ``ros_conversion``/``bag_converter``. The eight per-type modules are replaced by declarative
  group layouts.
* Fixed odometry conversion, which failed on every message (undefined ``timestamp``,
  ``twist.twist.twist`` access). ``pose_orientation`` now stores the full quaternion (4 values
  instead of 3) and the odometry attributes are filled instead of ``TODO``.
* Fixed the ``--force`` error message (undefined ``args``) and opening the output in append
  mode, which mixed old and new data on ``--force``.
* The storage plugin is detected automatically instead of assuming mcap.
* ``-t`` fails with the list of topics missing in the bag. Unsupported topics are reported once
  per topic with their message count at the end.
* Corrected attribute typos and units (e.g. IMU angular velocity ``rad/s`` instead of
  ``def/s``). Added ``--description`` for the file description (was a test text).
* Removed the ``progressbar2`` dependency; progress is printed when run in a terminal.
* License changed from Apache-2.0 to MIT, matching the repository. Added unit and end-to-end
  tests.
