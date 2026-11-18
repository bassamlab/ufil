# A supporting interpolation point for a planned path or trajectory.
# These points define the reference trajectory in a piecewise manner:
#  - Used in **path tracking mode** to represent spatially congruent waypoints
#    along the path, where 'distance' measures cumulative progress.
#  - Used in **trajectory mode** to represent time-parameterized waypoints
#    with both a target time and optionally target speed.

builtin_interfaces/Time time        # Target timestamp (used only in trajectory mode)
geometry_msgs/Pose pose             # Position and orientation at this support point (always required)
float64 distance                    # Cumulative path distance from the first point (only for path mode)
float64 speed                       # Intended traversing speed at this point (only for trajectory mode)
