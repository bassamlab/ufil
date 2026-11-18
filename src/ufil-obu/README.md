# Ufil Obu

ROS 2 packages of the UFIL on-board unit (OBU), part of the **UFIL** software stack.

| Package | Content |
| --- | --- |
| `ufil_obu_object_aggregation` | Publishes the own vehicle as a UFIL object in UTM coordinates (component `ufil_obu_object_aggregation::ObjectAggregationNode`) |
| `ufil_obu` | Launch files: `state_estimation.launch.xml` (robot_localization and object aggregation), `cam.launch.xml` (ETSI CAM via UDP) |

## Usage

```bash
ros2 launch ufil_obu state_estimation.launch.xml namespace:=obu \
  robot_localization_params_file:=/path/to/robot_localization.yaml \
  object_aggregation_params_file:=/path/to/vehicle.yaml
ros2 launch ufil_obu cam.launch.xml namespace:=obu target_ip:=192.168.0.10
```

## Documentation

For detailed documentation, please visit:

👉 [https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation](https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation)

## License

This project is licensed under the [MIT License](LICENSE).

