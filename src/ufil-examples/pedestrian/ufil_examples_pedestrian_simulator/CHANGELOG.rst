^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples_pedestrian_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The node is a composable component (``ufil_examples_pedestrian_simulator::PedestrianSimulatorNode``);
  the ``ufil_examples_pedestrian_simulator_node`` executable is generated from it. Node name
  changed to ``pedestrian_simulator``.
* ``scenario_file`` accepts a file name in the ``config`` directory of this package (default
  ``crossing.yaml``) besides an absolute path. ``simulation_timestep`` [ms, integer] became
  ``timestep`` [s]. Added ``frame_id`` (was hard-coded to ``map``). All parameters are
  read-only and validated.
* When the scenario ends, the node stops publishing and stays idle instead of shutting down the
  ROS 2 context (which would stop every node in a component container).
* A missing or invalid scenario file fails the start of the node with a fatal log message
  instead of a node that silently does nothing.
* The core library has a new ROS-independent API: ``ScenarioDescription`` /
  ``loadScenarioFile()`` / ``parseScenario()`` (throwing ``ScenarioError``), ``Agent`` built
  from waypoints, and ``PedestrianSimulator`` returning a ``SimulationStep`` per step.
  yaml-cpp is no longer part of the public headers. The scenario file format is unchanged.
* Added ``config/default.yaml``, ``launch/default.launch.xml``, diagnostics and unit tests.
  ``launch/pedestrian_simulator.launch.xml`` is kept as a wrapper with the former arguments.
* Removed the per-step INFO log message.
