^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_ssl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-11-18)
------------------
* ``launch/wheel_tracker.launch.xml`` starts the C++ component ``ssl_wheel_tracker`` and
  replaces ``wheel_detection.launch.xml`` and ``wheel_tracking.launch.xml``.
* ``launch/vehicle_tracking.launch.xml`` starts the C++ component ``ssl_vehicle_tracking`` and
  replaces ``vehicle_detection.launch.xml``.
* Depends on ``ssl_simulator`` (formerly ``ssl_instance_virtual``); ``simulation.launch.xml`` no
  longer has the ``simulation_enable_stats`` argument.
* Simon Schäfer is the sole maintainer; Lucas Hegerath is listed as author.

0.1.0 (2025-11-05)
------------------
* Initial version: meta package with the launch files of the SSL.
