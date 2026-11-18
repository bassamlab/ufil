^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for the UFIL workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The packages have their own changelogs in their components (``src/*``).

0.2.0 (2026-11-18)
------------------
* Releases are made by the ``release`` job (``ci/release.py``): run a pipeline on ``dev`` with
  ``RELEASE_VERSION`` set. It releases all components, this workspace and the documentation
  together and publishes the workspace and the documentation to GitHub.
* Moved from git.rwth-aachen.de to git.unibw.de; the CI runs on the Docker group runners.
* CI: ``build-test`` builds the packages one at a time and runs their tests (not those of the
  external ``carla_msgs``); ``rosdep-check`` fails if a ``package.xml`` declares a dependency that
  the Docker install scripts do not install.
* Docker: development image in ``docker/``, runtime setup with docker compose and profiles for the
  scenarios (``pedestrian``, ``carla-highway``, ``cpm-lab``, each also as ``-head`` and
  ``-headless`` except ``pedestrian``), ``bag``, ``foxglove-bridge`` and ``terminal``. The ROS
  middleware is selected with ``RMW_IMPLEMENTATION`` (default ``rmw_zenoh_cpp``, with a Zenoh
  router); ``BAG`` only has to be set for the ``bag`` profile.
* Lichtblick: new version with configured CORS origins; layouts for the pedestrian tracking and the
  CPM Lab; the deprecated Lichtblick extensions and layouts are removed.
