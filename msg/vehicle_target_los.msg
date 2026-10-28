uint64 timestamp

# ================================================================
# Target residual LOS angle relative to current camera optical axis
# Unit: rad
#
# When the gimbal tracks the target well,
# these values should stay near zero.
# ================================================================
float32 los_azimuth
float32 los_elevation

# ================================================================
# Inertial target LOS angular rate
# Unit: rad/s
#
# az: right positive
# el: up positive
# ================================================================
float32 los_az_rate
float32 los_el_rate

# ================================================================
# Gimbal attitude relative to aircraft body
# Unit: rad
#
# roll : right positive
# pitch: up positive
#
# These two angles describe the target LOS direction
# relative to the aircraft when the gimbal optical axis
# remains locked on the target.
# ================================================================
float32 gimbal_pitch
float32 gimbal_roll

# ================================================================
# Target detection / tracking valid
# ================================================================
bool target_valid
