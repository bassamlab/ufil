# Ufil Workspace

This is a ROS2 workspace developed as part of the **UFIL** software stack.

## Running an example

The workspace ships a `docker-compose.yaml` whose services are organised into profiles: one per example workload (`carla-highway`, `cpm-lab`, `pedestrian`), the `foxglove-bridge` visualization frontend, plus `router`, `terminal`, and `build`. Picking what to run is a matter of selecting profiles.

First time only, build the runtime image and compile the workspace inside it:

```
docker compose build foxglove-bridge
docker compose --profile build up
```

Then bring up an example together with the Foxglove bridge:

```
COMPOSE_PROFILES=carla-highway,foxglove-bridge docker compose up
COMPOSE_PROFILES=cpm-lab,foxglove-bridge docker compose up
COMPOSE_PROFILES=pedestrian,foxglove-bridge docker compose up
```

`carla-highway` and `cpm-lab` run the whole example on one machine. To split it across two machines like the real deployment, run the `-head` profile (on-board units and visualization) on one and the `-headless` profile (sensors, SSL, and RSU fusion) on the other, e.g. `carla-highway-head` and `carla-highway-headless`.

The `pedestrian` profile runs the simulator and tracker; pick a scenario via `PEDESTRIAN_SCENARIO` (default `crossing.yaml`), e.g. `PEDESTRIAN_SCENARIO=group.yaml COMPOSE_PROFILES=pedestrian,foxglove-bridge docker compose up`. All scenarios use the sidewalks and crosswalks of the example map:

| Scenario | Pedestrians | Tracker challenge |
|---|---|---|
| `crossing.yaml` | 2 enter at the west/east border, walk along the sidewalks, cross the junction diagonally (passing 1.2 m apart at its center), and leave along the far sidewalks; 2 more appear later | crossing paths, track start/end |
| `parallel.yaml` | 2 enter side by side at the south border, walk north along the west sidewalk and over the west crosswalk, and leave at the north border; 2 more appear later | closely spaced targets, track start/end |
| `passing.yaml` | 2 enter at opposite borders, walk towards each other along the east sidewalk, pass on the east crosswalk, and leave at the opposite border; 2 more appear later | opposing targets passing closely, track start/end |
| `waiting.yaml` | 2 wait at the curb (6 s and 3 s) before crossing; 1 walks past without stopping | stop-and-go, zero velocity |
| `group.yaml` | 4 walk in a tight 2 x 2 formation over the east crosswalk, then split up | association ambiguity, track splitting |
| `appearing.yaml` | 6 enter and leave the scene at different times | track initialisation and termination |
| `occlusion.yaml` | 2; one is undetected for 3 s mid-crosswalk, and 15 % of all detections are missed | coasting through gaps |
| `overtaking.yaml` | a jogger (3 m/s) overtakes a slow walker (1 m/s) and merges back in front of it | different speeds, lateral manoeuvre |
| `clutter.yaml` | 2, plus on average 2 false detections per step anywhere on the map | false positive rejection |
| `crowd.yaml` | 8 with staggered starts on all sidewalks and crosswalks, 1 to 2 m/s | many targets, opposing flows |

Scenario files support:
- per agent: either a `starting`/`stopping` pair or a list of timed `waypoints`; an agent only exists between its first and last waypoint time. Repeating a position with a later time makes it stand still. If `orientation` is omitted, the agent faces its walking direction (and keeps that heading while standing). Optional `occlusions` (`start`/`end` in seconds) suppress its detections.
- per scenario: `detection_probability` (default 1.0) for random missed detections, `clutter` (`mean_count` false detections per step, uniformly within `area: [x_min, y_min, x_max, y_max]`), and a `seed` for reproducible noise.

Ground truth (`reference_objects`) always contains every existing agent; missed detections and clutter only affect `measured_objects`.
The tracker launch also publishes a synthetic four-way Lanelet2 intersection on `/map` in the `map_utm` frame, using the shared visualization palette. The 40 x 40 m square contains a major east-west road (two 3 m lanes per direction) with a zebra crosswalk continuing each minor-road sidewalk across it, a minor north-south road (one 3 m lane per direction) whose approaches yield to the major road at stop lines, 2 m sidewalks on both sides of each road, and vegetation in the four corners. Roads have straight connections only and no traffic lights.
The local `map` origin is the southwest corner, and the intersection and pedestrian trajectories are centered at `(20, 20)`. The TF chain is `utm_32n -> map_utm -> map -> osn`: `map_utm` is translated to UTM32N `(696303.25, 5328527.39)` metres, and `map` is rotated 30 degrees about Z relative to `map_utm`. The OSM coordinates use the same translation and rotation, so map markers and pedestrian objects align in either frame. The layout is a demo, not a survey of that location.

All ROS services share one middleware, selected with `RMW_IMPLEMENTATION` (default `rmw_zenoh_cpp`; also `rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`) and `ROS_DOMAIN_ID` (default `21`):

```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp COMPOSE_PROFILES=pedestrian,foxglove-bridge docker compose up
```

With Zenoh, every ROS profile also starts the `router` service (published on port `7447`), and all nodes connect to it; with another RMW it exits right away. `COMPOSE_PROFILES=router` launches the router alone. To split an example across two machines, run the `-head` profile on one and point the other at its router, skipping the local one:

```
ZENOH_ROUTER=false ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<head-ip>:7447"];listen/endpoints=["tcp/0.0.0.0:0"]' COMPOSE_PROFILES=cpm-lab-headless docker compose up
```

Foxglove serves on `ws://localhost:8765`. All images and the devcontainer are headless: they contain no GUI tools (no RViz, no rqt), forward no X11 display, and install no recommended apt packages. Visualize through the bridge instead. Profiles compose freely, so any example can run with or without the bridge.

To view an example in [Lichtblick](https://github.com/lichtblick-suite/lichtblick), open the desktop app or a hosted instance, connect to `ws://localhost:8765` (Foxglove WebSocket), and import the matching layout from [layouts/](layouts/) (*Layouts → Import from file*). No extension is needed: every example converts its object lists into `visualization_msgs/MarkerArray` topics (`/osn/object_markers`, `/obu/object_markers`, `/rsu/object_markers`, `/rsu/cam/object_markers`, `/ssl/object_markers`) with `ufil_visualization`. Each source has its own colour:

| Source | Topic | Colour |
|---|---|---|
| OSN | `/osn/object_markers` | blue `#3584e4` |
| OBU | `/obu/object_markers` | green `#2ec27e` |
| CAM | `/rsu/cam/object_markers` | violet `#9141ac` |
| RSU | `/rsu/object_markers` | orange `#ff7800` |
| SSL | `/ssl/object_markers` | yellow `#f5c211` |

Each element of an object (`body`, `velocity`, `body_axis_x`/`y`/`z`, `covariance_position`, `covariance_orientation`, `label`) is a separate marker namespace, so it can be hidden per topic in the 3D panel's topic settings. Don't set a colour on these topics in Lichtblick; that would override the source colours. One `object_list_marker_array` node per example publishes all of these topics. Sources, topics and colours are configured in `src/ufil-core/ufil_visualization/config/ufil_sources.yaml`, and an example's own parameter file can override them (the pedestrian example, for instance, adds its ground truth and detections).

To replay a ros2 bag alongside an example, point the `bag` profile at the bag directory on the host:

```
BAG=/absolute/path/to/my_bag COMPOSE_PROFILES=cpm-lab-headless,bag,foxglove-bridge docker compose up
```

The path is bind-mounted read-only at `/bag` inside the container and replayed with `ros2 bag play`. `BAG` is only needed when the `bag` profile is active; without it the `bag` service exits with an error.

## Container images

All images are defined in [docker/Dockerfile](docker/Dockerfile):

| Stage | Used by |
|---|---|
| `runtime` | `docker-compose.yaml`. ROS 2 Jazzy (without the desktop meta-package) with all workspace dependencies. Built locally as `ufil-runtime:local`. |
| `dev` | The devcontainer (`.devcontainer/`). The `runtime` image plus a non-root user and dev tooling, so development, CI and deployment share the same dependencies. Built locally as `ufil-devcontainer:local`. |

Dependencies are pinned for reproducible builds:

- **Base image:** `ros:jazzy` is pinned by digest (`ROS_IMAGE` in the Dockerfile).
- **Python packages:** top-level packages go into [docker/requirements.txt](docker/requirements.txt); exact versions for amd64 and arm64 live in [docker/constraints.txt](docker/constraints.txt). After changing `requirements.txt`, or to update packages, run `docker/update-python-lock.sh` (requires Docker with buildx) and commit both files.
- **ROS and system packages:** add them to [docker/install-system.sh](docker/install-system.sh). The Dockerfile and CI both install from it (Python packages likewise through [docker/install-python.sh](docker/install-python.sh)). The `rosdep-check` CI job fails if a `package.xml` declares a dependency that is not installed.

## CI/CD

The pipeline ([.gitlab-ci.yml](.gitlab-ci.yml)) builds and tests the workspace. There is no container registry, so instead of a prebuilt image the jobs start from the pinned `ros:jazzy` base image (`ROS_IMAGE`, kept in sync with the Dockerfile) and install the dependencies with the scripts in [docker/](docker/). On tags, it pushes a flattened snapshot to the GitHub mirror.

## Documentation

For detailed documentation, please visit:

👉 [https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation](https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation)

## License

This project is licensed under the [MIT License](LICENSE).
