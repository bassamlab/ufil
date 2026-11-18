^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_viz_helpers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* Removed ``rviz.launch.xml`` and the ``rviz2`` dependency; UFIL data is visualized through
  ``foxglove-bridge.launch.xml`` only.
* ``foxglove-bridge.launch.xml`` passes ``use_sim_time`` to the bridge (it was ignored) and
  exposes ``port``.
