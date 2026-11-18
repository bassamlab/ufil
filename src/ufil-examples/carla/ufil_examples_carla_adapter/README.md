# ufil_examples_carla_adapter

Converts the ground-truth objects of the [CARLA ROS bridge](https://github.com/carla-simulator/ros-bridge)
into UFIL object lists. The latest actor list provides the classification, the axle geometry of
the known vehicle blueprints and the role names of the cooperative vehicles (`vehicle_<id>`).

- Core library `ufil_examples_carla_adapter` (`include/ufil_examples_carla_adapter/carla_adapter.hpp`),
  no ROS 2 dependencies.
- Component `ufil_examples_carla_adapter::CarlaAdapterNode`, executable
  `ufil_examples_carla_adapter_node`.

| Interface | Name | Description |
| --- | --- | --- |
| Subscription | `objects` | `derived_object_msgs/ObjectArray`, launch default `/carla/objects`. |
| Subscription | `actor_list` | `carla_msgs/CarlaActorList`, launch default `/carla/actor_list`. Objects are dropped until the first list arrives. |
| Publisher | `object_list` | `ufil_msgs/ObjectList` in `target_frame`. |
| Publisher | `/diagnostics` | Received, converted and dropped objects, transform errors. |
| Parameter | `target_frame` | Frame of the published lists (default `map`). |
| Parameter | `cooperative_vehicles.enabled` | `false` (default): every object is published. |
| Parameter | `cooperative_vehicles.ids` | IDs of the published cooperative vehicles; only declared if enabled. |

```bash
ros2 launch ufil_examples_carla_adapter default.launch.xml use_sim_time:=true
```
