# Command sent to a specific vehicle to control its motion.
# Depending on the selected mode, the car will interpret and execute the message differently.

# Target vehicle
uint8 vehicle_id                    # ID of the vehicle this command is for (must be set)

# Command types
uint8 INVALID = 0                   # Invalid command; vehicle should ignore
uint8 DIRECT = 1                    # Use motor_throttle and steering_servo directly
uint8 PATH_TRACKING = 2             # Follow given path[] at constant speed
uint8 SPEED_CURVATURE = 3           # Drive using speed + curvature control
uint8 TRAJECTORY = 4                # Follow full path[] as a time-based trajectory

int32 type                          # Mode used to interpret this command (one of the constants above)

# Header
std_msgs/Header header              # Should be set to frame_id="map"; stamp = message creation time

# Execution timing
builtin_interfaces/Time valid_after_stamp
                                    # Command becomes valid at this time (e.g. planner trigger time + max expected network latency)

# DIRECT control fields (only used if type == DIRECT)
float32 motor_throttle              # Range [-1.0, 1.0]; forward/backward throttle
float32 steering_servo              # Range [-1.0, 1.0]; left/right steering

# SPEED_CURVATURE control fields (only used if type == SPEED_CURVATURE)
float32 speed                       # Desired constant vehicle speed [m/s]
float32 curvature                   # Desired path curvature [1/m]

# PATH or TRAJECTORY control fields
PathPoint[] path                    # Path or trajectory supporting points.
                                    #
                                    # Each PathPoint represents a spatial pose and optionally:
                                    # - distance: cumulative path length from first point (PATH_TRACKING mode)
                                    # - speed: target speed at this point (TRAJECTORY mode)
                                    # - time: when the vehicle should be at this point (TRAJECTORY mode)
                                    #
                                    # The path is used differently depending on the `type`:
                                    #   - PATH_TRACKING: vehicle follows path[] using constant speed
                                    #   - TRAJECTORY: path[] includes timing and speed targets
