^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_obu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* ``state_estimation.launch.py`` and ``robot_localization.launch.py`` replaced by
  ``state_estimation.launch.xml``. The robot_localization parameter file is now a required
  argument (``robot_localization_params_file``); object parameters are passed as a YAML file.
* Fixed: ``navsat_transform_node`` subscribed to an unremapped ``imu`` topic.
* ``cam.launch.xml`` arguments lost their ``cam_`` prefix; ``use_sim_time`` defaults to false.
* Dependencies reduced to the packages used by the launch files; added a launch test.
