# Ufil Rsu

ROS 2 packages of the UFIL road side unit (RSU), part of the **UFIL** software stack.

| Package | Content |
| --- | --- |
| `ufil_central_fusion` | Fuses the object lists of several sensors (component `ufil_central_fusion::CentralFusionNode`) |
| `ufil_rsu` | Launch files: `central_fusion.launch.xml`, `cam.launch.xml` (ETSI CAM reception via UDP) |

## Usage

```bash
ros2 launch ufil_rsu central_fusion.launch.xml params_file:=/path/to/sensors.yaml
ros2 launch ufil_rsu cam.launch.xml listen_port:=5000
```

The fused sensors are configured in the parameter file, see
`ufil_central_fusion/config/default.yaml`.

## Documentation

For detailed documentation, please visit:

👉 [https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation](https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation)

## License

This project is licensed under the [MIT License](LICENSE).
