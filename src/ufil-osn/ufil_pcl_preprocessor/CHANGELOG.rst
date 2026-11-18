^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_pcl_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-29)
------------------
* The node is a composable component (``ufil_pcl_preprocessor::PclPreprocessorNode``); the
  ``ufil_pcl_preprocessor_node`` executable is generated from it.
* The core library no longer depends on ROS 2. ROS conversions live in ``component/``.
* Parameters are grouped (``filter.*``, ``fov.*``, ``occlusion_grid.*``), declared from the
  core defaults, validated as a whole and can be changed at runtime.
  Renamed: ``filter_min_number_of_neighbors`` (now an integer) to ``filter.min_neighbors``,
  ``use_ros_time`` to ``restamp_output``, ``depth_buffer_*`` to ``occlusion_grid.*``.
  Added ``sensor_frame`` (previously hard-coded ``sensor_node``).
  Removed unused ``depth_buffer_history_length``, ``depth_buffer_az_bins``,
  ``depth_buffer_el_bins`` and ``depth_buffer_preprocess_*``.
* Topics renamed: output ``cloud`` to ``dynamic_cloud``, ``filter_viz`` to ``filter_markers``.
* ``static_object_file`` defaults to empty; a configured file that cannot be read fails startup.
* Fixed: the radius outlier filter removed points by index position instead of the actual
  outliers.
* Fixed: occlusion grid resolution and size parameters had no effect.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
* Removed the vendored rapidcsv and the unused BoxClipper3D backport.
