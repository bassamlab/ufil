^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples_carla_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The node is a composable component (``ufil_examples_carla_adapter::CarlaAdapterNode``); the
  ``ufil_examples_carla_adapter_node`` executable is generated from it. The default node name is
  ``carla_adapter`` (was ``ufil_examples_carla_adapter_node``).
* The conversion moved into the ROS-independent core library ``ufil_examples_carla_adapter``
  (``CarlaAdapter``, ``CarlaAdapterConfig``, ``transformObjects()``); the ROS message
  conversions are in ``component/ufil_examples_carla_adapter_ros.hpp``.
* Topics are relative and remapped by the launch file: ``objects`` (``/carla/objects``),
  ``actor_list`` (``/carla/actor_list``) -> ``object_list``. The adapter uses the latest actor
  list for each object array instead of republishing the actor list with a timestamp on
  ``/carla/actor_list_stamped`` and synchronizing both; object arrays received before the
  first actor list are dropped.
* Removed the message ``CarlaActorListStamped`` that only served this synchronization.
* Parameters ``n_vehicles`` and ``vehicle_ids`` were replaced by ``cooperative_vehicles.enabled``
  (default ``false``: every object is published) and ``cooperative_vehicles.ids`` (only declared
  if enabled). ``target_frame`` is kept. All parameters are read-only and validated.
* A missing transform to ``target_frame`` is logged as throttled warning and reported in the
  diagnostics instead of an error per message.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
* Removed the unused UTM offset computation, static transform broadcaster and odometry
  publishers, and the unused dependencies on ``boost``, ``geographiclib``, ``message_filters``,
  ``nav_msgs``, ``sensor_msgs``, ``tf2_eigen``, ``tf2_geometry_msgs``, ``ufil_ros`` and
  ``rosidl_default_generators``.
