# Ufil Tools

This is a ROS2 package developed as part of the **UFIL** software stack.

## Packages

| Package | Kind | Usage |
|---|---|---|
| `ufil_bag_manipulator` | command line tool | `ufil_bag_manipulator merge BAG1 BAG2 [...] -o OUT`<br>`ufil_bag_manipulator trim BAG -o OUT --trim SECONDS [--keep-timestamps]` |
| `ufil_hdf_converter` | command line tool | `ufil_hdf_converter BAG [-o OUT.hdf5 \| -k KEY] [-t TOPIC ...] [-f]` |
| `ufil_pcl_map_builder` | ROS 2 node | `ros2 launch ufil_pcl_map_builder default.launch.xml` |
| `ufil_tools` | meta package | depends on all of the above |

The command line tools are on the `PATH` after `source install/setup.bash`; they are started
directly, not with `ros2 run`, and need no running ROS graph. Pass `--help` for all options.

`ufil_pcl_map_builder` stores every point published with the *Publish point* tool of the Foxglove or
Lichtblick 3D panel (topic `/clicked_point`) as a typed node of a Lanelet2 OSM file. Switch the type of the next point with
`ros2 param set /pcl_map_builder point_type tree`, or with the number keys when the node runs
in an interactive terminal (`ros2 run ufil_pcl_map_builder ufil_pcl_map_builder_node`).

## Documentation

For detailed documentation, please visit:

👉 [https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation](https://cpm.pages.rwth-aachen.de/Project/sfb-trr/software/documentation)

## License

This project is licensed under the [MIT License](LICENSE).

