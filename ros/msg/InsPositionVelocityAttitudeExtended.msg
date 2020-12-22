std_msgs/Header header

# Solution status
# 0: ins_inactive            1: ins_aligning
# 2: ins_high_variance       3: ins_solution_good
# 6: ins_solution_free       7: ins_alignment_complete
# 8: determining_orientation 9: waiting_initialpos
# 10: waiting_azimuth        11: initializing_biases
# 12: motion_detect
uint32 ins_status

# Position type
# 0: none           		1: fixedpos        		2: fixedheight
# 3: reserved       		4: floatconv       		5: widelane
# 6: narrowlane     		7: reserved        		8: doppler_velocity
# 9-15: reserved    		16: single         		17: psrdiff
# 18: waas          		19: propagated     		20-31: reserved
# 32: L1_float      		33: ionofree_float 		34: narrow_float
# 35-47: reserved   		48: L1_int	       		49: wide_int
# 50: narrow_int    		51: rtk_direct_ins 		52: ins_sbas
# 53: ins_psrsp     		54: ins_psrdiff    		55: ins_rtkfloat
# 56: ins_rtkfixed  		57-67: reserved    		68: ppp_converging
# 69: ppp           		70: operational    		71: warning
# 72: out_of_bounds 		73: ins_ppp_converging  74: ins
# 77: ppp_basic_converging  78: ppp_basic 			79: ins_ppp_basic
# 80: ins_ppp_basic_converging          
uint32 position_type

float64 latitude            # latitude (deg)
float64 longitude           # longitude (deg)
float64 height              # height above mean sea level (m)
float32 undulation          # Undulation - the relationship between the geoid and the ellipsoid (m) 
float64 north_velocity      # velocity in a northerly direction (m/s)
float64 east_velocity       # velocity in an easterly direction (m/s)
float64 up_velocity         # velocity in an up direction
float64 roll                # right handed rotation around y-axis (degrees)
float64 pitch               # right handed rotation aruond x-axis (degrees)
float64 azimuth             # right handed rotation around z-axis (degrees)
float32 latitude_std        # latitude standard deviation (m)
float32 longitude_std       # longitude standard deviation (m)
float32 height_std          # height standard deviation (m)
float32 north_velocity_std  # north velocity standard deviation (m/s)
float32 east_velocity_std   # east velocity standard deviation (m/s)
float32 up_velocity_std     # up velocity standard deviation (m/s)
float32 roll_std            # up velocity standard deviation (m/s)
float32 pitch_std           # up velocity standard deviation (m/s)
float32 azimuth_std         # up velocity standard deviation (m/s)
uint32 ext_solution_status    # Extended solution status
uint16 time_since_update    # Elapsed time since the last ZUPT or position update (s)   
