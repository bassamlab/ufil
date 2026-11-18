^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples_cpm_lab_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* ``CpmLabAdapterNode`` handles several vehicles: ``vehicle_id`` is replaced by ``vehicle_ids``,
  and the input topics come from ``vehicle_state_topic`` with ``{id}`` replaced (default
  ``/lab/vehicle_{id}/vehicleState``) instead of remapping ``vehicle_state``. Each vehicle is
  published alone on ``vehicle_<id>/object_list`` on every state, and all vehicles with a state
  within ``state_timeout`` (0.5 s) together on ``object_list`` at ``publish_rate`` (20 Hz), so a
  visualization no longer flickers between the vehicles.
* The three nodes are composable components of the ``ufil_examples_cpm_lab_adapter_component``
  library; the executables ``ufil_examples_cpm_lab_adapter_node``,
  ``ufil_examples_cpm_lab_adapter_sensor_node`` and ``ufil_examples_osc_message_converter_node``
  are generated from them:

  * ``ufil_examples_cpm_lab_adapter::CpmLabAdapterNode`` (default name ``cpm_lab_adapter``,
    previously ``ufil_example_cpm_lab_adapter``) publishes a lab vehicle as a UFIL object list.
  * ``ufil_examples_cpm_lab_adapter::CpmLabSensorNode`` (``cpm_lab_sensor``, previously
    ``lab_adapter_node``) simulates GNSS, IMU and wheel odometry of a lab vehicle.
  * ``ufil_examples_cpm_lab_adapter::OscLaneControlLightNode`` (``osc_lane_control_light``,
    previously ``osc_publisher_node``) forwards lane control lights as JSON in OSC messages.

* Scaling, object conversion, sensor simulation (artificial GNSS, UTM conversion, outdated state
  handling) and the lane control light JSON encoding moved into the ROS-independent core library
  ``ufil_examples_cpm_lab_adapter``; ``message_converter.hpp`` was replaced by
  ``lane_control_light.hpp``.
* Replaced oscpack by the OSC sender of ``ufil_osc``: the oscpack 1.1.0 of Ubuntu aborts on 64 bit
  non-x86 platforms. ``cmake/FindOSCPACK.cmake`` was removed. Messages that do not fit into one
  UDP datagram and socket errors are reported instead of throwing from a fixed stack buffer.
* Parameters: ``id`` was renamed to ``vehicle_id`` (an integer in [0, 255] for both nodes; the
  sensor node declared it as a string before, so integer launch arguments failed). Sensor node:
  ``frequency_gnss``, ``frequency_odometry`` and ``use_artificial_gnss`` became
  ``gnss.frequency``, ``odometry.frequency`` and ``gnss.artificial_errors``; the GNSS origin is
  configurable with ``gnss.origin.latitude`` / ``gnss.origin.longitude``. Adapter node: the
  hard-coded frame ``lab`` is the ``source_frame`` parameter. OSC node: ``host`` and ``port``
  became ``receiver.host`` and ``receiver.port``, the ``source_topic`` parameter was replaced by
  the ``lane_control_lights`` topic and the default OSC address is ``/lane_control_light``. All
  parameters are read-only and validated; invalid values are logged as fatal and fail the
  construction.
* Added ``config/*.yaml`` and ``launch/*.launch.xml`` for each node.
* Fixed: the artificial GNSS read an uninitialised z coordinate of the receiver position.
* Fixed: the IMU yaw added ``eulerAngles(0, 1, 2).z()`` of the target frame rotation, which is not
  the yaw angle for every rotation; it now uses the yaw of the rotation.
* Fixed: the lane control light JSON escapes quotes, backslashes and control characters.
* The sensor timers use the node clock, so they follow ``use_sim_time``.
* GNSS fixes are logged at DEBUG instead of INFO, transform failures as throttled warnings.
  Added diagnostics to all three nodes and unit tests. Removed the unused oscpack, global compiler
  flag and ``ament_export_libraries`` set-up and the lint skips.
