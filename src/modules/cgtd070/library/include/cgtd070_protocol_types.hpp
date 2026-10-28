#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cgtd070 {

constexpr size_t kFeedbackFrameSize = 64U;
constexpr size_t kControlFrameSize = 44U;
constexpr uint8_t kFeedbackHeader0 = 0xFCU;
constexpr uint8_t kFeedbackHeader1 = 0x2CU;
constexpr uint8_t kControlHeader0 = 0xFBU;
constexpr uint8_t kControlHeader1 = 0x2CU;
constexpr uint8_t kFrameTail = 0xF0U;
constexpr uint8_t kJsonPayloadType = 0xFFU;

enum class Command : uint8_t {
    Idle = 0x00U,
    CancelTracking = 0x3BU,
    Center = 0x71U,
    GreenBoxTracking = 0x93U,
};

enum class FrameKind : uint8_t {
    Telemetry = 0U,
    Json,
    Unknown,
};

enum class DecodeError : uint8_t {
    None = 0U,
    NullInput,
    WrongLength,
    BadHeader,
    BadTail,
    BadChecksum,
};

// Flight-controller telemetry embedded in bytes 8..37 of the 44-byte
// control frame. Integer fields already use the protocol's wire units so the
// transport-independent core does not depend on PX4 attitude/GPS types.
// A zero-initialized value is the protocol-defined "no flight data" value.
struct FlightTelemetry {
    int16_t pitch_cdeg;
    int16_t roll_cdeg;
    uint16_t heading_cdeg;

    uint8_t gps_year_since_2000;
    uint8_t gps_month;
    uint8_t gps_day;
    uint8_t gps_hour;
    uint8_t gps_minute;
    uint8_t gps_second;
    uint8_t gps_centisecond;

    float longitude_deg;
    float latitude_deg;
    uint8_t satellites_used;
    float gps_altitude_m;
    uint16_t battery_decivolt;
    uint16_t relative_height_decimeter;
};

struct Telemetry {
    uint8_t payload_type;
    uint8_t self_test;
    uint16_t status_primary;
    uint16_t status_secondary;
    uint8_t servo_state;

    float gimbal_azimuth_deg;
    float gimbal_pitch_deg;
    float gimbal_roll_deg;
    uint8_t version;
    uint16_t laser_range_m;
    uint16_t frame_counter;

    uint8_t tf_used_percent;
    uint8_t tf_total_gb;
    float attitude_pitch_deg;
    float attitude_roll_deg;

    uint8_t target_id;
    uint8_t target_type;
    float target_longitude_deg;
    float target_latitude_deg;
    int16_t target_altitude_m;

    float gyro_azimuth_rate_dps;
    float gyro_pitch_rate_dps;
    float gyro_roll_rate_dps;
    uint8_t displayed_image;
    float digital_zoom;

    float target_los_azimuth_rate_dps;
    float target_los_pitch_rate_dps;
    int16_t tracking_box_width_px;
    int16_t tracking_box_height_px;
    float miss_azimuth_deg;
    float miss_pitch_deg;
};

struct Feedback {
    FrameKind kind;
    uint8_t payload_type;
    Telemetry telemetry;
    char json[60];
    size_t json_length;
};

struct ParserStats {
    uint32_t bytes_received;
    uint32_t frames_ok;
    uint32_t checksum_errors;
    uint32_t framing_errors;
    uint32_t feedback_overruns;
};

} // namespace cgtd070
