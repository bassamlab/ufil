# Observation of a vehicle’s current pose as detected by the infrastructure positioning system (IPS).
# This message is published by the IPS and consumed by components such as planners or controllers.

uint8 vehicle_id                    # ID of the observed vehicle

std_msgs/Header header              # Standard ROS2 header
                                    # - frame_id: frame in which the pose is given (should be "map")
                                    # - stamp: time when this message was created by the observer

builtin_interfaces/Time valid_after_stamp
                                    # Time at which the pose was considered valid
                                    # This allows the observer to indicate a precise point in time when the pose was accurate
                                    # E.g., the camera or localization pipeline latency is compensated here

geometry_msgs/Pose pose             # 6DOF pose of the vehicle in the map frame (position + orientation)
