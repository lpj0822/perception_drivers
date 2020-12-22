std_msgs/Header header

# obstacle id
uint8 obj_id

# indicates which mode updated the fused track
# 0: no MR/LR update
# 1: MR update only
# 2: LR update only
# 3: both MR/LR update
uint8 med_range_mode

# measurement status
# 0: no target
# 1: new target
# 2: new updated target
# 3: updated target
# 4: coasted target
# 5: merged target
# 6: invalid coasted target
# 7: new coasted target
uint8 status

# azimuth (degree)
# 0 = toward front of vehicle parallel to vehicle centerline, (+) = clockwise
# factor 0.1, range -51.2 to 51.1 
float32 angle

# range (m)
# (+) = away from sensor
# factor 0.1, range 0 to 204.7
float32 range

# range acceleration (m/s^2)
# (+) = away from sensor
# factor 0.05, range -25.6 to 25.5
float32 range_accel

# range rate (m/s)
# (+) = away from sensor
# factor 0.01, range -81.92 to 81.91
float32 range_rate

# width (m)
# factor 0.5, range 0 to 7.5
float32 width

# lateral rate (m/s)
# (+) = counter clockwise
# factor 0.25, range -8 to 7.75
float32 lat_rate

# indicates if target is a bridge or not
# 0: not a bridge
# 1: bridge
bool is_bridge

# oncoming flag
# 0: not oncoming
# 1: oncoming
bool is_oncoming

# indicates that number of detections associated with the track have changed
# 0: no change
# 1: change
bool is_grouping_changed