std_msgs/Header header

uint8 obj_id

# longitudinal coordinate (m)
# factor 0.2, range -500 to 1138.2
float32 obj_dist_long

# lateral coordinate (m)
# factor 0.2, range -204.6 to 204.8
float32 obj_dist_lat

# relative velocity in longitudinal direction (m/s)
# factor 0.25, range -128 to 127.75
float32 obj_vrel_long

# relative velocity in lateral direction (m/s)
# factor 0.25, range -64 to 63.75
float32 obj_vrel_lat

# dynamic property of the object indicating if the object is moving or stationary
# 0: moving
# 1: stationary
# 2: oncoming
# 3: stationary candidate
# 4: unknown
# 5: crossing stationary
# 6: crossing moving
# 7: stopped
uint8 obj_dyn_prop

# radar cross section (dBm^2)
# factor 0.5, range -64 to 64.5
float32 obj_rcs

# standard deviation of longitudinal distance
float32 obj_dist_long_rms

# standard deviation of lateral distance
float32 obj_dist_lat_rms

# standard deviation of longitudinal relative velocity
float32 obj_vrel_long_rms

# standard deviation of lateral relative velocity
float32 obj_vrel_lat_rms

# standard deviation of longitudinal relative acceleration
float32 obj_arel_long_rms

# standard deviation of lateral relative acceleration
float32 obj_arel_lat_rms

# standard deviation of orientation angle
float32 obj_orientation_rms

# measurement state indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
# 0: deleted object
# 1: new object is created
# 2: measured
# 3: predicted
# 4: deleted for merge
# 5: new form merge
uint8 obj_meas_state

# probability of existence
# 0: invalid
# 1: < 25%
# 2: < 50%
# 3: < 75%
# 4: < 90%
# 5: < 99%
# 6: < 99.9%
# 7: <= 100%
uint8 obj_prob_of_exist

# relative acceleration in longitudinal direction (m/s^2)
# factor 0.01, range -10 to 10.47
float32 obj_arel_long

# relative acceleration in lateral direction (m/s^2)
# factor 0.01, range -2.5 to 2.61
float32 obj_arel_lat

# object classification
# 0: point
# 1: car
# 2: truck
# 3: not in use
# 4: motorcycle
# 5: bicycle
# 6: wide
# 7: reserved
uint8 obj_class

# orientation angle of the object (deg)
# factor 0.4, range -180 to 180
float32 obj_orientation_angle

# length of the tracked object (m)
# factor 0.2, range 0 to 51
float32 obj_length

# width of the tracked object (m)
# factor 0.2, range 0 to 51
float32 obj_width