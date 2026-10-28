# Minimal CGTD070 feedback consumed by guidance. The protocol core still
# decodes every V2.12 field; only these six values cross uORB.
uint64 timestamp

float32 los_az_rate    # target azimuth LOS rate, deg/s
float32 los_el_rate    # target elevation LOS rate, deg/s
float32 gimbal_roll    # gimbal azimuth/frame angle, deg (requested name)
float32 gimbal_pitch   # gimbal pitch frame angle, deg
float32 los_azimuth    # target miss angle in azimuth direction, deg
float32 los_elevation  # target miss angle in elevation direction, deg
bool target_valid
