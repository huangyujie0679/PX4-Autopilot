# pn_guidance guidance-command ULog diagnostics
# Units: acceleration m/s^2, velocity m/s, quaternion [w,x,y,z]

uint64 timestamp
uint32 seq
uint16 version

float32 vc_raw
float32 vc_filtered
float32 vc_used

float32[3] ekf_velocity
float32 ekf_speed

float32 speed_target
float32 axial_used
float32 axial_limit
float32 speed_brake
float32 bpn_scale

float32[3] bpn_raw_body
float32[3] bpn_used_body
float32[3] accel_cmd_ned

float32[4] q_raw
float32[4] q_slew
float32[4] q_cmd
float32[4] q_actual

float32 thrust_preview
float32 thrust_out

uint8 state
uint8 rig_stage
uint8 vc_valid
uint8 velocity_valid
uint8 guidance_valid
uint8 slew_limited
uint8 lead_limited
uint8 cage_limited
uint8 alt_floor_active
