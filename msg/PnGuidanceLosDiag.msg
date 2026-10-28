# pn_guidance LOS/observer ULog diagnostics
# Units: timestamp us, angle rad, angular rate rad/s unless field says otherwise

uint64 timestamp
uint32 seq
uint16 version

uint64 los_sample_time
uint64 los_rx_time
uint64 source_timestamp
uint64 capture_time
uint32 capture_age_us
uint32 vision_frame_seq
int32 body_rate_align_error_us

float32 los_sample_age_ms
float32 recon_dt_ms
float32 att_sync_dt_ms
float32 body_rotation_deg

float32[2] gimbal_angle
float32[2] gimbal_delta
float32[2] gimbal_rate_raw
float32[2] gimbal_rate_filt
float32[2] miss_angle
# 【V21修改】RK3576黑盒LOS角速度，仅用于诊断/对照，不再参与真机observer
float32[2] raw_los_rate
# 【V21修改】PX4根据direct miss_angle历史自行重构的正式IMG角速度
float32[2] miss_rate_px4
# 【V21修改】PX4 miss-rate中心差分总窗口，单位ms；当前为40ms
float32 miss_rate_window_ms
# 【V21修改】PX4 miss-rate是否有效
uint8 miss_rate_valid

float32[3] los_body
float32[3] omega_aligned
float32[3] los_dot_img
float32[3] los_dot_gim
float32[3] los_dot_body
float32[3] los_dot_fraw
float32[3] los_dot_in

uint8 target_valid
uint8 los_fresh
uint8 attitude_valid
uint8 inertial_rate_valid
uint8 body_rate_valid
uint8 body_rate_interpolated
uint8 los_time_fallback
uint8 capture_time_valid
uint8 observer_initialized
uint8 reacquire_active
