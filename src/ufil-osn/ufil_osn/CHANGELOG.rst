^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_osn
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* Replaced ``pcl_preprocessor.launch.xml`` and ``osn_tracker.launch.xml`` with
  ``osn.launch.xml``, which composes both components into one container with intra-process
  communication. Parameters are passed as YAML files instead of one argument per parameter.
* Added a launch test running a synthetic point cloud through the pipeline.
