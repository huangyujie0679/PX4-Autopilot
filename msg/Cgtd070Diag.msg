uint64 timestamp

# 三层接收时间
uint64 last_rx_byte_time
uint64 last_parsed_frame_time
uint64 last_telemetry_time

# 当前时刻距离上述事件的时间，单位 ms
# 0xFFFFFFFF 表示从启动以来尚未发生
uint32 rx_byte_age_ms
uint32 parsed_frame_age_ms
uint32 telemetry_age_ms

# 发送/接收统计
uint32 tx_frames
uint32 rx_frames
uint32 telemetry_frames
uint32 io_errors
uint32 missed_tx_slots

# UART driver状态
int32 serial_open_errno
int32 serial_read_errno
int32 serial_write_errno
uint32 serial_read_errors
uint32 serial_write_errors
uint32 serial_eagain

# poll状态
int32 poll_errno
uint32 poll_errors

# parser状态
uint32 parser_bytes
uint32 parser_ok
uint32 parser_checksum
uint32 parser_framing
uint32 parser_overruns

# 最近一次Telemetry帧自身的信息
uint32 feedback_counter
uint32 feedback_status_primary
uint32 feedback_status_secondary
uint32 feedback_servo_state

bool have_rx_bytes
bool have_parsed_frame
bool have_telemetry
