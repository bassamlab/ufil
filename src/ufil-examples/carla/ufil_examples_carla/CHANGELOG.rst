^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples_carla
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* Per scenario (``highway``, ``intersection``) only four launch files remain:
  ``carla-<scenario>_head.launch.xml`` (CARLA adapter and area filter in ``/obu``, on-board units
  ``/obu_0`` to ``/obu_6`` sending CAMs, visualization), ``carla-<scenario>_headless.launch.xml``
  (outdoor sensor node, sensitive surface layer, road side unit with CAM reception and central
  fusion), ``carla-<scenario>.launch.xml``, which starts both on a single machine, and
  ``carla-<scenario>_simulation.launch.xml`` (CARLA adapter, area filter and virtual SSL mat for
  recorded runs).
* All parameters moved into ``config/carla-<scenario>.yaml`` (one section per node). The launch
  files only set namespaces, topics and the URDFs, and pass ``use_sim_time`` to all nodes.
  ``carla-<scenario>.launch.xml`` loads ``config/carla-<scenario>_local.yaml`` on top, which
  sends the CAMs to the local RSU (``127.0.0.1``) instead of the headless machine
  (``192.168.1.41``). Both files can be replaced with the ``params_file`` and
  ``local_params_file`` arguments.
* The nodes are started directly instead of through the wrapper launch files of ``ufil_obu``,
  ``ufil_rsu``, ``ufil_osn``, ``ufil_ssl``, ``ufil_visualization``, ``ufil_message_filter``,
  ``ufil_udp``, ``ufil_etsi`` and ``ufil_examples_carla_adapter``. Node names and topics are
  unchanged, except that the ETSI conversion nodes are named ``cam_encoder`` (vehicles) and
  ``cam_decoder`` (RSU) instead of ``etsi_its_conversion_node``.
* The object marker nodes moved from the headless to the head launch files; the headless machine
  no longer starts any visualization. The single-machine launch files now also start the marker
  node of the SSL objects (``object_list_marker_array_ssl``), and the ``/obu`` marker node of
  ``carla-highway_head.launch.xml`` shows ``/obu/object_list`` (filtered) instead of
  ``/obu/object_list_unfiltered``, like all other launch files.
* The SSL uses ``ssl_wheel_tracker`` and ``ssl_vehicle_tracking`` (C++) with the ``car_carla``
  template of ``ssl_vehicle_tracking``, which is identical to the removed copy
  ``config/car_carla.json``. The SSL simulator now loads the ``ssl_simulator`` defaults
  (``frame_period``); all values set before are unchanged.
* Removed the sub-launch files (``central_fusion``, ``obu``, ``obu_simulation``, ``osn``,
  ``rsu``, ``ssl``, ``ssl_simulation``) and the unused ``enable_stats`` argument.
  ``config/osn_pcl_preprocessor.yaml`` and ``config/simulation_rsu.yaml`` were merged into
  ``config/carla-<scenario>.yaml``. The dependencies are now the packages whose executables and
  configuration files are used (``ufil_obu``, ``ufil_osn``, ``ufil_rsu`` and ``ufil_ssl`` were
  replaced by the component packages).
* The OBU launch files (``obu.launch.xml``, ``obu_simulation.launch.xml``) start the CARLA
  adapter through ``ufil_examples_carla_adapter/launch/default.launch.xml`` (node
  ``/obu/carla_adapter``, all simulator objects, frame ``map``). Removed ``adapter.launch.xml``,
  which set parameters the adapter never declared (``enable_stats``, ``frequency_*``) and could
  not be launched on its own.
* Removed ``enable_stats`` where it had no effect: the argument of ``central_fusion.launch.xml``
  and ``obu_simulation.launch.xml`` and the value passed to ``osn.launch.xml``. ``ssl.launch.xml``
  now passes ``enable_stats`` to the SSL nodes.
* Every include is wrapped in a scoped group, so the arguments of one included launch file no
  longer leak into the following includes.
* Added argument descriptions, a meaningful package description, the MIT license, the
  dependencies of all used launch files and lint tests. Removed the unused build dependencies
  on ``ufil_core`` and ``ufil_examples_carla_adapter``.
* The simulator scripts pass ``flake8`` and ``pep257`` (formatting and docstrings only).
