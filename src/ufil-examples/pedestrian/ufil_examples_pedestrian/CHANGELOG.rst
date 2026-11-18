^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples_pedestrian
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* Only three launch files remain: ``pedestrian_head.launch.xml`` (transforms and visualization of
  the map, the ground truth, the detections and the tracks), ``pedestrian_headless.launch.xml``
  (pedestrian simulator and pedestrian tracker of the outdoor sensor node) and
  ``pedestrian.launch.xml``, which starts both on a single machine. ``use_sim_time`` (default
  ``false``) and ``scenario`` (default ``crossing.yaml``) of ``pedestrian.launch.xml`` are
  unchanged; ``pedestrian_headless.launch.xml`` has the ``scenario`` argument too.
* All parameters moved into ``config/pedestrian.yaml`` (one section per node): the map and its
  frame of the Lanelet2 visualization and, explicitly, the default timestep and frame of the
  simulator and the default tracker settings. The launch files only set namespaces, topics, the
  URDF, the scenario file and ``use_sim_time``. The file can be replaced with the
  ``params_file`` argument; ``local_params_file`` (default: ``params_file``) is loaded on top for
  machine-specific overrides. There are no single-machine overrides, so there is no
  ``config/pedestrian_local.yaml``.
* The nodes are started directly instead of through the ``default.launch.xml`` files of
  ``ufil_examples_pedestrian_simulator`` and ``ufil_examples_pedestrian_tracker`` and the
  ``object_list`` / ``lanelet_map`` launch files of ``ufil_visualization``. Node names,
  namespaces and topics are unchanged.
* ``robot_state_publisher`` (``/osn``), which only serves the visualization, runs on the head
  machine; before, it was part of the tracker launch file.
* Removed the sub-launch files ``simulation.launch.xml`` and ``tracker.launch.xml``.
* The launch files include the ``default.launch.xml`` files of the reworked simulator and tracker
  and the ``object_list`` / ``lanelet_map`` launch files of ufil_visualization, each in its own
  group so that no launch arguments leak between them. Arguments and topics are unchanged.
* The ``scenario`` argument also accepts an absolute path to a scenario file.
* Dependencies are declared as ``exec_depend`` on the packages whose launch files and
  executables are used; the unused ``ufil_core`` dependency was removed.
* Added the lint tests.
