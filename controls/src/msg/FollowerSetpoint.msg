uint8 leader_seq_num 0 # sequence number of the leader that sent the message (default 0)
float32[4] q # Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
float32 thrust # Upward thrust (in body frame)
float32 aviata_yaw_est # Yaw estimate of the originator of this setpoint (radians)
uint8 aviata_docking_slot # Docking slot of the originator of this setpoint

# Unused:
# float32 body_roll_rate # [rad/s] Body roll rate
# float32 body_pitch_rate # [rad/s] Body pitch rate
# float32 body_yaw_rate # [rad/s] Body yaw rate
