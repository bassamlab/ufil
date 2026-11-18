^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples_cpm_lab
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The head machine runs a single ``/obu/cpm_lab_adapter`` for all vehicles instead of two adapters
  per vehicle. Its list of all vehicles feeds ``/obu/object_list``; the on-board unit of each
  vehicle reads ``/obu/vehicle_<id>/object_list``.
* Only three launch files remain: ``cpm-lab_head.launch.xml`` (on-board units of the vehicles 12,
  13, 15, 17, 18, 19 and the visualization), ``cpm-lab_headless.launch.xml`` (outdoor sensor node,
  sensitive surface layer, road side unit with CAM reception and central fusion) and
  ``cpm-lab.launch.xml``, which starts both on a single machine.
* All parameters moved into ``config/cpm-lab.yaml`` (one section per node). The launch files only
  set namespaces, topics and the URDFs, and pass ``use_sim_time``. ``cpm-lab.launch.xml`` loads
  ``config/cpm-lab_local.yaml`` on top, which sends the CAMs to the local RSU (``127.0.0.1``)
  instead of the headless machine (``192.168.1.41``). Both files can be replaced with the
  ``params_file`` and ``local_params_file`` arguments.
* The nodes are started directly instead of through the wrapper launch files of ``ufil_obu``,
  ``ufil_rsu``, ``ufil_osn``, ``ufil_ssl``, ``ufil_visualization``, ``ufil_message_filter`` and
  ``ufil_udp``. Node names and topics are unchanged, except that the ETSI conversion nodes are
  named ``cam_encoder`` (vehicles) and ``cam_decoder`` (RSU).
* The SSL uses ``ssl_wheel_tracker`` and ``ssl_vehicle_tracking`` (C++) with the
  ``muCar_scaled_lab`` template of ``ssl_vehicle_tracking``.
* Removed the sub-launch files (``central_fusion``, ``cpm_lab_adapter``, ``obu``, ``osn``,
  ``rsu``, ``single_obu``, ``ssl``), ``obu_sensors.launch.xml`` with its ``param`` directory, the
  OSC forwarding (``osc-mapper``, ``osc_lane_control_lights``) and the unused ``enable_stats``
  argument. ``config/osn_pcl_preprocessor.yaml``, ``config/sfb_demonstrator_rsu.yaml`` and the
  vehicle template copy were merged into or replaced by ``config/cpm-lab.yaml``.
* Version 0.2.0, license MIT and a package description. The package only has ``exec_depend``
  dependencies on the packages whose launch files and executables it uses (the ``ufil_core``
  metapackage was replaced by ``ufil_message_filter``, ``ufil_osc`` and ``ufil_visualization``;
  added ``ufil_osn``, ``ufil_ssl``, ``robot_state_publisher``, ``launch_ros`` and
  ``launch_xml``). Added the ament lint tests.
* ``obu_sensors.launch.py`` was replaced by ``obu_sensors.launch.xml``. It starts one vehicle
  (``vehicle_id`` instead of the ``ids`` list, which XML cannot iterate) and includes the new
  ``cpm_lab_sensor.launch.xml`` of ``ufil_examples_cpm_lab_adapter``; the Python file included a
  ``sensors.launch.py`` that did not exist. ``publish_world_frames`` disables the world frame
  publisher when the file is included for several vehicles.
* ``cpm_lab_adapter.launch.xml`` and ``osc_lane_control_lights.launch.xml`` include the launch
  files of ``ufil_examples_cpm_lab_adapter`` with the reworked parameters; their arguments are
  unchanged except for the removed ``cpm_lab_adapter_enable_stats``, which the adapter never
  declared. The adapters are named ``cpm_lab_adapter_<vehicle id>``: the six adapters in the
  ``obu`` namespace had the same name before. ``osc_lane_control_lights.launch.xml`` gained
  ``use_sim_time``.
* Fixed: the lanelet map visualization did not receive ``use_sim_time``.
* Removed ``enable_stats`` arguments passed to ``osn.launch.xml``, ``ssl.launch.xml`` and
  ``central_fusion.launch.xml``, which do not use them.
