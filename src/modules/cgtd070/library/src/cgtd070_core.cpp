#include "cgtd070_core.hpp"

#include <string.h>

namespace cgtd070 {
namespace {

uint16_t read_u16_le(const uint8_t *data) noexcept
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

int16_t read_i16_le(const uint8_t *data) noexcept
{
    return static_cast<int16_t>(read_u16_le(data));
}

float read_f32_le(const uint8_t *data) noexcept
{
    const uint32_t bits = static_cast<uint32_t>(data[0]) |
                          (static_cast<uint32_t>(data[1]) << 8U) |
                          (static_cast<uint32_t>(data[2]) << 16U) |
                          (static_cast<uint32_t>(data[3]) << 24U);
    float value = 0.0F;
    static_assert(sizeof(value) == sizeof(bits), "32-bit float required");
    memcpy(&value, &bits, sizeof(value));
    return value;
}

float scaled_i16(const uint8_t *data) noexcept
{
    return static_cast<float>(read_i16_le(data)) * 0.01F;
}

void put_u16_le(uint8_t *data, uint16_t value) noexcept
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void put_i16_le(uint8_t *data, int16_t value) noexcept
{
    put_u16_le(data, static_cast<uint16_t>(value));
}

int16_t constrain_joystick_rate(int16_t value) noexcept
{
    if (value < -100) {
        return -100;
    }
    if (value > 100) {
        return 100;
    }
    return value;
}

void put_f32_le(uint8_t *data, float value) noexcept
{
    uint32_t bits = 0U;
    static_assert(sizeof(value) == sizeof(bits), "32-bit float required");
    memcpy(&bits, &value, sizeof(bits));
    data[0] = static_cast<uint8_t>(bits & 0xFFU);
    data[1] = static_cast<uint8_t>((bits >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((bits >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((bits >> 24U) & 0xFFU);
}

bool valid_command(Command command) noexcept
{
    switch (command) {
    case Command::Idle:
    case Command::CancelTracking:
    case Command::Center:
    case Command::YawFollowAircraft:
    case Command::GreenBoxTracking:
        return true;
    }
    return false;
}

} // namespace

Core::Core() noexcept
{
    reset();
}

void Core::reset() noexcept
{
    memset(rx_frame_, 0, sizeof(rx_frame_));
    memset(&latest_feedback_, 0, sizeof(latest_feedback_));
    memset(&stats_, 0, sizeof(stats_));
    rx_used_ = 0U;
    pending_command_ = Command::Idle;
    last_error_ = DecodeError::None;
    feedback_available_ = false;
}

void Core::ingest_rx(const uint8_t *data, size_t length) noexcept
{
    if (data == nullptr) {
        if (length != 0U) {
            last_error_ = DecodeError::NullInput;
        }
        return;
    }

    for (size_t i = 0U; i < length; ++i) {
        ++stats_.bytes_received;
        push_byte(data[i]);
    }
}

bool Core::take_feedback(Feedback &output) noexcept
{
    if (!feedback_available_) {
        return false;
    }

    memcpy(&output, &latest_feedback_, sizeof(output));
    feedback_available_ = false;
    return true;
}

bool Core::request_command(Command command) noexcept
{
    if (!valid_command(command)) {
        return false;
    }

    if (command == Command::Idle) {
        pending_command_ = Command::Idle;
        return true;
    }

    if (pending_command_ != Command::Idle) {
        return false;
    }

    pending_command_ = command;
    return true;
}

void Core::build_next_tx_frame(uint8_t output[kControlFrameSize]) noexcept
{
    const FlightTelemetry telemetry{};
    const JoystickControl joystick{};
    build_next_tx_frame(output, telemetry, joystick);
}

void Core::build_next_tx_frame(
    uint8_t output[kControlFrameSize],
    const FlightTelemetry &telemetry) noexcept
{
    const JoystickControl joystick{};
    build_next_tx_frame(output, telemetry, joystick);
}

void Core::build_next_tx_frame(
    uint8_t output[kControlFrameSize],
    const FlightTelemetry &telemetry,
    const JoystickControl &joystick) noexcept
{
    if (output == nullptr) {
        return;
    }

    memset(output, 0, kControlFrameSize);
    output[0] = kControlHeader0;
    output[1] = kControlHeader1;
    output[2] = static_cast<uint8_t>(pending_command_);

    // For 93H green-box tracking, protocol parameter 1 remains zero.
    // Frame byte numbers below are one-based, as written in protocol V2.12.
    put_i16_le(output + 7U, telemetry.pitch_cdeg);       // bytes 8..9
    put_i16_le(output + 9U, telemetry.roll_cdeg);        // bytes 10..11
    put_u16_le(output + 11U, telemetry.heading_cdeg);    // bytes 12..13
    output[13] = telemetry.gps_year_since_2000;          // byte 14
    output[14] = telemetry.gps_month;
    output[15] = telemetry.gps_day;
    output[16] = telemetry.gps_hour;
    output[17] = telemetry.gps_minute;
    output[18] = telemetry.gps_second;
    output[19] = telemetry.gps_centisecond;              // byte 20
    put_f32_le(output + 20U, telemetry.longitude_deg);   // bytes 21..24
    put_f32_le(output + 24U, telemetry.latitude_deg);    // bytes 25..28
    output[28] = telemetry.satellites_used;              // byte 29
    put_f32_le(output + 29U, telemetry.gps_altitude_m);  // bytes 30..33
    put_u16_le(output + 33U, telemetry.battery_decivolt);// bytes 34..35
    put_u16_le(output + 35U,
               telemetry.relative_height_decimeter);    // bytes 36..37
    if (joystick.enabled != 0U) {
        output[37] = 0x70U;                              // byte 38
        put_i16_le(output + 38U,
                   constrain_joystick_rate(joystick.azimuth_rate_dps));
        put_i16_le(output + 40U,
                   constrain_joystick_rate(joystick.pitch_rate_dps));
    }
    output[42] = control_checksum(output);
    output[43] = kFrameTail;
    pending_command_ = Command::Idle;
}

uint8_t Core::feedback_checksum(const uint8_t *frame) noexcept
{
    if (frame == nullptr) {
        return 0U;
    }

    uint8_t checksum = 0U;
    for (size_t i = 2U; i <= 61U; ++i) {
        checksum ^= frame[i];
    }
    return checksum;
}

uint8_t Core::control_checksum(const uint8_t *frame) noexcept
{
    if (frame == nullptr) {
        return 0U;
    }

    uint8_t checksum = 0U;
    for (size_t i = 2U; i <= 41U; ++i) {
        checksum ^= frame[i];
    }
    return checksum;
}

DecodeError Core::decode_frame(const uint8_t *data,
                               size_t length,
                               Feedback &output) noexcept
{
    if (data == nullptr) {
        return DecodeError::NullInput;
    }
    if (length != kFeedbackFrameSize) {
        return DecodeError::WrongLength;
    }
    if (data[0] != kFeedbackHeader0 || data[1] != kFeedbackHeader1) {
        return DecodeError::BadHeader;
    }
    if (data[63] != kFrameTail) {
        return DecodeError::BadTail;
    }
    if (feedback_checksum(data) != data[62]) {
        return DecodeError::BadChecksum;
    }

    memset(&output, 0, sizeof(output));
    output.payload_type = data[2];

    if (data[2] == kJsonPayloadType) {
        output.kind = FrameKind::Json;
        size_t length_out = 0U;
        while (length_out < 59U && data[3U + length_out] != 0U) {
            output.json[length_out] = static_cast<char>(data[3U + length_out]);
            ++length_out;
        }
        output.json[length_out] = '\0';
        output.json_length = length_out;
        return DecodeError::None;
    }

    // V2.12 documents 01H..04H. Deployed devices also emit 00H using the
    // same telemetry layout, so 00H remains accepted for compatibility.
    if (data[2] > 4U) {
        output.kind = FrameKind::Unknown;
        return DecodeError::None;
    }

    output.kind = FrameKind::Telemetry;
    Telemetry &t = output.telemetry;
    t.payload_type = data[2];
    t.self_test = data[3];
    t.status_primary = read_u16_le(data + 4U);
    t.status_secondary = read_u16_le(data + 6U);
    t.servo_state = data[8];
    t.gimbal_azimuth_deg = scaled_i16(data + 9U);
    t.gimbal_pitch_deg = scaled_i16(data + 11U);
    t.gimbal_roll_deg = scaled_i16(data + 13U);
    t.version = data[15];
    t.laser_range_m = read_u16_le(data + 16U);
    t.frame_counter = read_u16_le(data + 18U);
    t.tf_used_percent = data[21];
    t.tf_total_gb = data[22];
    t.attitude_pitch_deg = scaled_i16(data + 23U);
    t.attitude_roll_deg = scaled_i16(data + 25U);
    t.target_id = data[27];
    t.target_type = data[28];
    t.target_longitude_deg = read_f32_le(data + 29U);
    t.target_latitude_deg = read_f32_le(data + 33U);
    t.target_altitude_m = read_i16_le(data + 37U);
    t.gyro_azimuth_rate_dps = scaled_i16(data + 41U);
    t.gyro_pitch_rate_dps = scaled_i16(data + 43U);
    t.gyro_roll_rate_dps = scaled_i16(data + 45U);
    t.displayed_image = data[47];
    t.digital_zoom = static_cast<float>(data[48]) * 0.1F;
    t.target_los_azimuth_rate_dps = scaled_i16(data + 50U);
    t.target_los_pitch_rate_dps = scaled_i16(data + 52U);
    t.tracking_box_width_px = read_i16_le(data + 54U);
    t.tracking_box_height_px = read_i16_le(data + 56U);
    t.miss_azimuth_deg = scaled_i16(data + 58U);
    t.miss_pitch_deg = scaled_i16(data + 60U);
    return DecodeError::None;
}

void Core::push_byte(uint8_t byte) noexcept
{
    if (rx_used_ == 0U) {
        if (byte == kFeedbackHeader0) {
            rx_frame_[0] = byte;
            rx_used_ = 1U;
        }
        return;
    }

    if (rx_used_ == 1U) {
        if (byte == kFeedbackHeader1) {
            rx_frame_[1] = byte;
            rx_used_ = 2U;
        } else if (byte == kFeedbackHeader0) {
            rx_frame_[0] = byte;
        } else {
            rx_used_ = 0U;
        }
        return;
    }

    rx_frame_[rx_used_] = byte;
    ++rx_used_;
    if (rx_used_ < kFeedbackFrameSize) {
        return;
    }

    last_error_ = decode_frame(rx_frame_, kFeedbackFrameSize, latest_feedback_);
    if (last_error_ == DecodeError::None) {
        accept_frame();
        rx_used_ = 0U;
        return;
    }

    if (last_error_ == DecodeError::BadChecksum) {
        ++stats_.checksum_errors;
    } else {
        ++stats_.framing_errors;
    }
    resynchronize();
}

void Core::accept_frame() noexcept
{
    if (feedback_available_) {
        ++stats_.feedback_overruns;
    }
    feedback_available_ = true;
    ++stats_.frames_ok;
}

void Core::resynchronize() noexcept
{
    for (size_t start = 1U; start + 1U < kFeedbackFrameSize; ++start) {
        if (rx_frame_[start] == kFeedbackHeader0 &&
            rx_frame_[start + 1U] == kFeedbackHeader1) {
            const size_t remaining = kFeedbackFrameSize - start;
            memmove(rx_frame_, rx_frame_ + start, remaining);
            rx_used_ = remaining;
            return;
        }
    }

    if (rx_frame_[kFeedbackFrameSize - 1U] == kFeedbackHeader0) {
        rx_frame_[0] = kFeedbackHeader0;
        rx_used_ = 1U;
    } else {
        rx_used_ = 0U;
    }
}

} // namespace cgtd070
