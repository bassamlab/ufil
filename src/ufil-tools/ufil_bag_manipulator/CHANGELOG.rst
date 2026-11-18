^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_bag_manipulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* The tool is the ``ufil_bag_manipulator`` command on the ``PATH`` after sourcing the
  workspace, with the subcommands ``merge`` and ``trim``. It replaces the ``ros2 run``
  executables ``merge_rosbags`` and ``trim_shift_rosbags``.
* ``merge`` accepts two or more input bags and rejects topics with conflicting types.
* ``trim`` crashed on start (undefined ``messages_info``); fixed. ``--keep-timestamps`` keeps
  the original timestamps instead of shifting the cutoff to 0.
* The input storage (mcap, sqlite3) is detected automatically; the output storage is chosen
  with ``--storage`` and defaults to mcap (was sqlite3 for both).
* Merge and trim logic live in the ROS-independent ``bag_operations`` module; rosbag2 access
  is isolated in ``bag_io``. No ``rclpy.init`` is needed anymore.
* License changed from Apache-2.0 to MIT, matching the repository. Added unit and end-to-end
  tests.
