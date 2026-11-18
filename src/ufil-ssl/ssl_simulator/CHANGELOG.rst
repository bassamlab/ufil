^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ssl_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* Package renamed from ``ssl_instance_virtual`` to ``ssl_simulator``; the executable
  ``ssl_virtual_component_node`` is now ``ssl_simulator_node``.
* The node is a composable component (``ssl_simulator::SslSimulatorNode``); the
  ``ssl_simulator_node`` executable is generated from it. The default node name changed
  from ``ssl_publisher`` to ``ssl_simulator``.
* The simulation moved into the ROS-independent ``MatSimulator`` class with its own
  configuration, vehicle and pressure map types. Frames are simulated by a ROS timer on the node
  clock instead of a free-running thread, so ``use_sim_time`` works without the former time jump
  handler; a backward time jump drops the vehicles received before the jump.
* Fixed: the mat heading was declared as ``h`` while the launch files pass ``heading``, so the
  heading was silently ignored. The parameter is now ``heading`` [rad].
* Fixed: ``v_x`` / ``v_y`` of ``ufil_msgs/State`` are absolute velocity components. The old model
  used ``v_x`` as the speed along the heading and ignored ``v_y``; vehicles are now extrapolated
  with the absolute velocity and the yaw rate (``ufil_object_tracking`` extended constant
  velocity model).
* Added the ``frame_period`` parameter (default 0.023 s, the former fixed 23 ms). All parameters
  are read-only and validated; invalid values are logged as fatal and fail the construction.
  ``threshold`` is limited to [1, 255].
* The noise and pressure distribution use a random generator per simulator instance, seedable in
  the core library.
* Vehicles are rasterized whenever a wheel touches the mat, not only if their center lies within
  a safety margin around the mat. A wheel contact covers every cell whose center lies inside its
  square contact patch, also at the far mat border; overlapping contributions (wheels, line
  crosstalk) keep the maximum.
* Objects with a non-finite state are ignored. An object list without stamp is extrapolated from
  its reception time.
* Removed the ``kalman_cpp`` extended Kalman filter, the hardware driver interface without effect
  in the simulator (line filter, noise filter, ADC resolution, ...), the vendored ``gcem`` library
  and the ``fmt``, ``Eigen``, ``boost`` and ``tf2_geometry_msgs`` dependencies.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
* Removed the outdated ``Doxyfile`` and the empty ``doc`` directory.
