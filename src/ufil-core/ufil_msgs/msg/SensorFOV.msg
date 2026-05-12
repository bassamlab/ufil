# Cone-shaped Field of View (FOV) description for a directional sensor.
#
# The FOV is modeled as a 3D cone (or truncated cone) centered on the sensor's
# optical axis:
#  - r_min / r_max   : radial distance limits
#  - phi_min/phi_max : horizontal angular limits (left–right)
#  - theta_min/theta_max : vertical angular limits (up–down)
#
# The header's frame_id must contain the coordinate frame in which
# the FOV estimation was performed.

std_msgs/Header header

# Origin of the sensor in the FOV frame [m]
geometry_msgs/Point origin  

# Minimum valid sensing distance [m]
float32 r_min

# Maximum valid sensing distance [m]
float32 r_max

# Minimum horizontal angle (azimuth), negative = left [rad]
float32 phi_min

# Maximum horizontal angle (azimuth), positive = right [rad]
float32 phi_max

# Minimum vertical angle (elevation), negative = downward [rad]
float32 theta_min

# Maximum vertical angle (elevation), positive = upward [rad]
float32 theta_max

# Covariance matrix of FOV parameters (r_min, r_max, phi_min, phi_max, theta_min, theta_max)
# Flattened row-major order: 6x6 = 36 elements
float32[36] covariance
