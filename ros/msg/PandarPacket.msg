PandarBlock[6] blocks

# packet header
uint16 header

# indiacte if lidar temprature is unnomally high
# 0x00: normal               0x01: high temperature
uint8 high_temperature

# motor speed (rpm)
uint16 motor_speed

# the timestamp of first laser in first block (us)
uint32 block_timestamp

# return laser beam mode
# 0x37: strongest return mode (normally used)
# 0x38: last return mode
# 0x39: dual return mode
uint8 return_mode

# factory info (normally 0x42 or 0x43)
uint8 factory_info