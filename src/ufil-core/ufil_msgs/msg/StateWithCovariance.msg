# This expresses an object’s state
State state

# Row-major representation of the 8x8 state covariance matrix with the corresponding unit.
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, v_x, v_y, a_x, a_y, yaw, yaw_rate)
float64[64] covariance
