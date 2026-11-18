# ufil_message_filter

Forwards only the objects of a `ufil_msgs/ObjectList` whose position (`state.state.x/y`) lies
strictly inside a polygonal area of interest. The area is given in the frame of the input lists.

- Core library `ufil_message_filter` (`include/ufil_message_filter/position_filter.hpp`), no ROS 2
  dependencies.
- Component `ufil_message_filter::ObjectListPositionFilterNode`, executable
  `object_list_position_filter_node`.

| Interface | Name | Description |
| --- | --- | --- |
| Subscription | `objects` | Input object lists. |
| Publisher | `objects_filtered` | Header of the input list with the objects inside the area. Lists without such an object are not published. |
| Publisher | `/diagnostics` | Counters of received, passed and filtered objects. |
| Parameter | `area_of_interest.enabled` | `false` (default): every object is forwarded. |
| Parameter | `area_of_interest.vertices` | `[x1, y1, x2, y2, ...]` [m], at least 3 vertices; only declared if enabled. |

```bash
ros2 launch ufil_message_filter default.launch.xml params_file:=/path/to/area.yaml
ros2 launch ufil_message_filter object_list_position_filter.launch.xml \
  object_list_position_filter_area_of_interest:="[0.0, 0.0, 10.0, 0.0, 10.0, 5.0, 0.0, 5.0]"
```
