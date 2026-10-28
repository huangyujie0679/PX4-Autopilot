# V21.4 predicted-LOS causal LOS-rate estimator shadow diagnostic
# Shadow only: none of these candidates feeds BPN, attitude, thrust, or motor control.
# Vector rate unit is 1/s (numerically rad/s for small angular motion).

uint64 timestamp
uint32 seq
uint16 version

# Current-time target used by V21.2 prediction; same time basis as pn_guidance_prediction_diag.prediction_time
uint64 prediction_time

# Total currently retained predicted-LOS history span
float32 history_span_ms

# Actual span and linear-fit residual for each causal trailing window
float32 fit60_span_ms
float32 fit100_span_ms
float32 fit140_span_ms
float32 fit60_resid
float32 fit100_resid
float32 fit140_resid

# V21.2 constant-inertial-rate baseline, transported to current prediction time
float32[3] rate_const_ned
float32[3] rate_const_body

# Causal local-linear-regression derivatives of current-time predicted LOS direction
float32[3] rate_fit60_ned
float32[3] rate_fit60_body
float32[3] rate_fit100_ned
float32[3] rate_fit100_body
float32[3] rate_fit140_ned
float32[3] rate_fit140_body

# Conservative complementary candidates based on fit100:
# blend25 = 75% const + 25% fit100
# blend50 = 50% const + 50% fit100
float32[3] rate_blend25_ned
float32[3] rate_blend25_body
float32[3] rate_blend50_ned
float32[3] rate_blend50_body

uint8 history_count
uint8 fit60_samples
uint8 fit100_samples
uint8 fit140_samples
uint8 fit60_valid
uint8 fit100_valid
uint8 fit140_valid
uint8 estimator_valid
