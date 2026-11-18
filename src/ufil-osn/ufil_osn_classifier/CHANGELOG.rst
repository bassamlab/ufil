^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_osn_classifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* Default topics follow the OSN tracker: input ``track_input`` is ``osn/object_list`` (was
  ``pole/tracks``), output ``track_output`` is ``osn/classified_object_list`` (was
  ``pole/fused_tracks``).
* New parameter ``model_file``; it defaults to the installed ``models/yolo11s.engine``. The YOLO
  models moved from the Python module to ``models/``.
* The lidar-to-camera transforms ``T_camera_right_lidar`` / ``T_camera_left_lidar`` are parsed
  as nested number lists and validated instead of being passed to ``eval``.
* Status messages go through the ROS logger instead of ``print``; invalid confidences are
  logged as errors.
* Added ``config/default.yaml``, ``launch/default.launch.xml`` and unit tests for the confidence
  fusion and the matrix parameters.
