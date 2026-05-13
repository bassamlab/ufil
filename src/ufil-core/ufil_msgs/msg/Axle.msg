# Whether this axle only has one wheel (single track) or multiple (dual track)
bool single_track

# Width of the axle (only applicable if not single track)
float32 track_width # [m]
float32 track_width_variance # [m^2]

# Distance from the position of the vehicle (in most cases the center point) to this axle.
# If the axle is located in front of the center point, this value is positive, otherwise negative.
float32 center_to_axle # [m]
float32 center_to_axle_variance # [m^2]

# Detected wheels. One element if single_track is true, two elements for pessanger cars or four in case of
# twin tires. True if the corresponding wheel was detected.
# Direction: Outer left to outer right side
bool[] wheel_detected

# If you have no estimate for one of the data elements, set the element to 0 and the associated variance to -1.
# If you have an estimate for a data element, but no estimate for the variance, set it to 0.
# If you are interpreting this message, please check for these cases and treat them accordingly.