std_msgs/Header header

# obstacle id
uint8 obj_id

# indicates the motion pattern of the object
# 0: unknown
# 1: moving
# 2: reserved
uint8 object_motion_pattern

# indicates the type of the object
# 0: vehicle
# 1: bike
# 2: pedestrian
# 3: unknown
uint8 object_type

# alive time of the object
# factor 1, range 0 to 127 
uint8 alive_time

# confidence of being an object
# factor 7, range 0 to 100 
uint8 confidence

# longitudinal position (m)
# factor 0.1, range -127 to 127
float32 longitudinal_position

# lateral position (m)
# factor 0.1, range -102.3 to 102.3
float32 lateral_position

# longitudinal velocity (m/s)
# factor 0.1, range -51.1 to 51.2
float32 longitudinal_velocity

# lateral velocity (m/s)
# factor 0.1, range -51.1 to 51.2
float32 lateral_velocity

# longitudinal acceleration (m/s^2)
# factor 0.1, range -25.5 to 25.6
float32 longitudinal_acceleration

# lateral acceleration (m/s^2)
# factor 0.1, range -6.4 to 6.4
float32 lateral_acceleration

# width (m)
# factor 0.1, range 0 to 5
float32 width

# heading (degree)
# factor 1, range -180 to 180
float32 heading

# indicates if object is coasted or not
# 0: not coasted
# 1: coasted
bool is_coasted
