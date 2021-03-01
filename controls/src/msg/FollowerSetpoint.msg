float32[4] q # Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
float32 thrust # Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
uint8 leader_increment 0 # ID label from the sender (default 0)

# Unused:
# float32 body_roll_rate # [rad/s] Body roll rate
# float32 body_pitch_rate # [rad/s] Body pitch rate
# float32 body_yaw_rate # [rad/s] Body yaw rate
