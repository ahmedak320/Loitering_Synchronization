# Customized statesharing message
# to be added to the PX4 messages folder in the proper way
int64 timestamp
string frame_id
# global position coordinates
float32 global_position_lon
float32 global_position_lat
float32 global_position_alt
# local position
float32 local_position_x
float32 local_position_y
float32 local_position_z
# cartesian angles based on front right down x,y,z axis
float32 roll
float32 pitch
float32 yaw
# quaternion
float32[4] q
