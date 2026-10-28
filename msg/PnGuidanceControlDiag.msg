# pn_guidance PX4 inner-loop / rig ULog diagnostics
# Units: angular rate rad/s, ages ms

uint64 timestamp
uint32 seq
uint16 version

float32[3] rate_sp
float32[3] rate_actual
float32[3] torque_sp
float32[3] thrust_sp
float32[4] motor

float32 rates_sp_age_ms
float32 torque_sp_age_ms
float32 thrust_sp_age_ms
float32 motor_age_ms

float32 aux1
float32 attitude_error_deg
float32 rig_att_from_anchor_deg
float32 max_motor_abs

uint8 state
uint8 rig_stage
uint8 armed
uint8 rig_run
uint8 rig_fault
uint8 control_published
uint8 rc_valid
uint8 control_attitude_enabled
uint8 control_rates_enabled
