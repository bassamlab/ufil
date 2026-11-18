^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_rsu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* Launch arguments lost their ``cam_`` / ``central_fusion_`` prefixes; ``use_sim_time``
  defaults to false. Removed the unused ``enable_stats`` and costmap arguments of the fusion.
* Fixed: received CAMs were published on ``cam/objects`` while the fusion subscribes to
  ``cam/object_list``.
* Dependencies reduced to the packages used by the launch files; added a launch test.
