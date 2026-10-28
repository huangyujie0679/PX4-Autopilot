# V21.2 current-time prediction shadow 诊断消息
# 仅用于Shadow验证：本topic中的任何预测量都不反馈BPN、姿态设定、推力或电机控制。
# 单位：timestamp/observer_time/prediction_time 为us；prediction_dt_ms为ms；LOS为单位方向向量；LOS方向导数为1/s（数值等价rad/s小角速度量级）。

uint64 timestamp
uint32 seq
uint16 version

# 当前observer状态对应的真实测量时刻，与pn_guidance_los_diag.los_sample_time相同时间基准
uint64 observer_time
# prediction shadow外推到的目标时刻；当前实现取本次Run()可获得的最新vehicle_attitude.timestamp_sample（通常只比HRT now早几毫秒）
uint64 prediction_time
float32 prediction_dt_ms

# 姿态历史取样时刻相对observer_time的误差；插值成功时为0
int32 attitude_align_error_us

# observer时刻的LOS/LOS-rate
float32[3] los_observer_body
float32[3] los_observer_ned
float32[3] los_dot_observer_ned

# 采用“一阶恒惯性LOS-rate”传播到prediction_time后的结果
float32[3] los_predicted_ned
float32[3] los_predicted_body
float32[3] los_dot_predicted_ned
float32[3] los_dot_predicted_body

uint8 prediction_valid
uint8 attitude_interpolated
