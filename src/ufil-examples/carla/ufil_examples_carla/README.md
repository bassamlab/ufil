# ufil_examples_carla

UFIL example with the [CARLA](https://carla.org) simulator. The simulator, the CARLA ROS bridge
and the traffic manager run in docker (`simulator/<scenario>/docker-compose.yml`); the launch
files of this package start the UFIL stack on the simulated data.

Per scenario (`highway`, `intersection`):

| Launch file | Description |
| --- | --- |
| `carla-<scenario>_head.launch.xml` | Head machine: CARLA adapter with area filter (`/obu`), CAM senders of the vehicles 0 to 6 (`/obu_<i>`) to the RSU at 192.168.1.41 and the visualization. |
| `carla-<scenario>_headless.launch.xml` | Headless machine: OSN, SSL and RSU with CAM reception and central fusion. |
| `carla-<scenario>.launch.xml` | Head and headless on a single machine; the CAMs are sent to 127.0.0.1. |
| `carla-<scenario>_simulation.launch.xml` | Simulated sensors for recorded runs: CARLA adapter with area filter and a virtual SSL mat fed with the CARLA objects, with visualization. |

All parameters are in `config/carla-<scenario>.yaml`, one section per node. The single-machine
launch file loads `config/carla-<scenario>_local.yaml` on top. All launch files take
`use_sim_time` (default `true`, the CARLA ROS bridge publishes `/clock`), `params_file` and
`local_params_file`.

```bash
ros2 launch ufil_examples_carla carla-highway_headless.launch.xml
```
