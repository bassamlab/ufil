# Object ID
uint32 id

# Object state 
State state

# Object dimension 
Dimension dimension

# Object classification
# Indices of classes in the the classification vector
uint8 CAR = 0
uint8 TRUCK = 1
uint8 MOTORCYCLE = 2
uint8 BICYCLE = 3
uint8 PEDESTRIAN = 4
uint8 STATIONARY = 5
uint8 OTHER = 6

# Classification Vector
float32[7] classification
