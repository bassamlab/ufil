# Ufil Osn

ROS 2 packages of the UFIL outdoor sensor node (OSN), part of the **UFIL** software stack.

| Package | Content |
| --- | --- |
| `ufil_pcl_preprocessor` | Ground, outlier and static-area removal, field-of-view estimation and occlusion grid (component `ufil_pcl_preprocessor::PclPreprocessorNode`) |
| `ufil_osn_tracker` | Multi-object tracking on the dynamic point cloud (component `ufil_osn_tracker::OsnTrackerNode`) |
| `ufil_osn_classifier` | Camera-based classification of the tracks |
| `ufil_osn` | Launch file of the complete pipeline |

The C++ packages separate a ROS-independent core library (`include/`, `src/`) from the ROS 2
component (`component/`). Default parameters are in each package's `config/default.yaml`.

## Usage

```bash
ros2 launch ufil_osn osn.launch.xml input_topic:=/lidar/points use_sim_time:=true \
  pcl_preprocessor_params_file:=/path/to/overrides.yaml
```

Runs both components in the `/osn` namespace. The parameter files are loaded on top of the
package defaults, so they only need to contain the values that differ.

## Documentation

For detailed documentation, please visit:

👉 [https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation](https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation)

## License

This project is licensed under the [MIT License](LICENSE).

