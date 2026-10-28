#include "cgtd070_core.hpp"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

void put_u16(uint8_t *frame, size_t offset, uint16_t value)
{
    frame[offset] = static_cast<uint8_t>(value & 0xFFU);
    frame[offset + 1U] = static_cast<uint8_t>(value >> 8U);
}

void put_i16(uint8_t *frame, size_t offset, int16_t value)
{
    put_u16(frame, offset, static_cast<uint16_t>(value));
}

void put_f32(uint8_t *frame, size_t offset, float value)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    frame[offset] = static_cast<uint8_t>(bits & 0xFFU);
    frame[offset + 1U] = static_cast<uint8_t>((bits >> 8U) & 0xFFU);
    frame[offset + 2U] = static_cast<uint8_t>((bits >> 16U) & 0xFFU);
    frame[offset + 3U] = static_cast<uint8_t>((bits >> 24U) & 0xFFU);
}

bool near_value(float a, float b, float epsilon = 0.001F)
{
    return fabsf(a - b) < epsilon;
}

float get_f32(const uint8_t *frame, size_t offset)
{
    float value = 0.0F;
    memcpy(&value, frame + offset, sizeof(value));
    return value;
}

void make_telemetry(uint8_t frame[cgtd070::kFeedbackFrameSize], uint16_t counter)
{
    memset(frame, 0, cgtd070::kFeedbackFrameSize);
    frame[0] = cgtd070::kFeedbackHeader0;
    frame[1] = cgtd070::kFeedbackHeader1;
    frame[2] = 0U;
    frame[3] = 0xC3U;
    put_u16(frame, 4U, 0x13D1U);
    put_u16(frame, 6U, 0x0003U);
    frame[8] = 0x07U;
    put_i16(frame, 9U, -1234);
    put_i16(frame, 11U, 567);
    put_i16(frame, 13U, -89);
    frame[15] = 0x21U;
    put_u16(frame, 16U, 1234U);
    put_u16(frame, 18U, counter);
    frame[21] = 42U;
    frame[22] = 64U;
    put_i16(frame, 23U, -250);
    put_i16(frame, 25U, 375);
    frame[27] = 42U;
    frame[28] = 1U;
    put_f32(frame, 29U, 114.123456F);
    put_f32(frame, 33U, 30.123456F);
    put_i16(frame, 37U, 88);
    put_i16(frame, 41U, 123);
    put_i16(frame, 43U, -234);
    put_i16(frame, 45U, 345);
    frame[47] = 2U;
    frame[48] = 35U;
    put_i16(frame, 50U, -45);
    put_i16(frame, 52U, 67);
    put_i16(frame, 54U, 320);
    put_i16(frame, 56U, 240);
    put_i16(frame, 58U, -12);
    put_i16(frame, 60U, 34);
    frame[62] = cgtd070::Core::feedback_checksum(frame);
    frame[63] = cgtd070::kFrameTail;
}

void test_decode_fields()
{
    uint8_t frame[cgtd070::kFeedbackFrameSize];
    make_telemetry(frame, 65535U);
    cgtd070::Feedback output{};
    assert(cgtd070::Core::decode_frame(frame, sizeof(frame), output) ==
           cgtd070::DecodeError::None);
    assert(output.kind == cgtd070::FrameKind::Telemetry);
    assert(output.telemetry.payload_type == 0U);
    assert(output.telemetry.self_test == 0xC3U);
    assert(output.telemetry.status_primary == 0x13D1U);
    assert(output.telemetry.status_secondary == 3U);
    assert(output.telemetry.servo_state == 7U);
    assert(near_value(output.telemetry.gimbal_azimuth_deg, -12.34F));
    assert(near_value(output.telemetry.gimbal_pitch_deg, 5.67F));
    assert(output.telemetry.frame_counter == 65535U);
    assert(output.telemetry.target_id == 42U);
    assert(output.telemetry.target_type == 1U);
    assert(near_value(output.telemetry.target_longitude_deg, 114.123456F, 0.0001F));
    assert(output.telemetry.tracking_box_width_px == 320);
    assert(output.telemetry.tracking_box_height_px == 240);
    assert(near_value(output.telemetry.miss_pitch_deg, 0.34F));
}

void test_stream_chunks_noise_and_recovery()
{
    uint8_t good[cgtd070::kFeedbackFrameSize];
    uint8_t bad[cgtd070::kFeedbackFrameSize];
    make_telemetry(good, 100U);
    memcpy(bad, good, sizeof(bad));
    bad[10] ^= 0x01U;

    cgtd070::Core core;
    const uint8_t noise[] = {0x00U, 0xFCU, 0x01U, 0xAAU};
    core.ingest_rx(noise, sizeof(noise));
    core.ingest_rx(bad, 17U);
    core.ingest_rx(bad + 17U, sizeof(bad) - 17U);
    core.ingest_rx(good, 3U);
    core.ingest_rx(good + 3U, 29U);
    core.ingest_rx(good + 32U, sizeof(good) - 32U);

    cgtd070::Feedback output{};
    assert(core.take_feedback(output));
    assert(output.telemetry.frame_counter == 100U);
    assert(!core.take_feedback(output));
    assert(core.stats().frames_ok == 1U);
    assert(core.stats().checksum_errors == 1U);
}

void test_latest_frame_overrun()
{
    uint8_t first[cgtd070::kFeedbackFrameSize];
    uint8_t second[cgtd070::kFeedbackFrameSize];
    make_telemetry(first, 1U);
    make_telemetry(second, 2U);

    cgtd070::Core core;
    core.ingest_rx(first, sizeof(first));
    core.ingest_rx(second, sizeof(second));
    assert(core.stats().feedback_overruns == 1U);

    cgtd070::Feedback output{};
    assert(core.take_feedback(output));
    assert(output.telemetry.frame_counter == 2U);
}

void verify_tx(cgtd070::Core &core, cgtd070::Command command)
{
    uint8_t frame[cgtd070::kControlFrameSize];
    if (command != cgtd070::Command::Idle) {
        assert(core.request_command(command));
    }
    core.build_next_tx_frame(frame);
    assert(frame[0] == 0xFBU);
    assert(frame[1] == 0x2CU);
    assert(frame[2] == static_cast<uint8_t>(command));
    for (size_t i = 3U; i <= 41U; ++i) {
        assert(frame[i] == 0U);
    }
    assert(frame[42] == static_cast<uint8_t>(command));
    assert(frame[42] == cgtd070::Core::control_checksum(frame));
    assert(frame[43] == 0xF0U);
}

void test_control_state_machine()
{
    cgtd070::Core core;
    verify_tx(core, cgtd070::Command::Idle);
    verify_tx(core, cgtd070::Command::GreenBoxTracking);
    verify_tx(core, cgtd070::Command::Idle);
    verify_tx(core, cgtd070::Command::Center);
    verify_tx(core, cgtd070::Command::Idle);
    verify_tx(core, cgtd070::Command::CancelTracking);
    verify_tx(core, cgtd070::Command::Idle);

    assert(core.request_command(cgtd070::Command::Center));
    assert(!core.request_command(cgtd070::Command::CancelTracking));
    uint8_t frame[cgtd070::kControlFrameSize];
    core.build_next_tx_frame(frame);
    assert(frame[2] == 0x71U);
}

void test_control_flight_telemetry()
{
    cgtd070::Core core;
    cgtd070::FlightTelemetry telemetry{};
    telemetry.pitch_cdeg = -1234;
    telemetry.roll_cdeg = 5678;
    telemetry.heading_cdeg = 35999U;
    telemetry.gps_year_since_2000 = 26U;
    telemetry.gps_month = 8U;
    telemetry.gps_day = 12U;
    telemetry.gps_hour = 9U;
    telemetry.gps_minute = 10U;
    telemetry.gps_second = 11U;
    telemetry.gps_centisecond = 42U;
    telemetry.longitude_deg = 120.129623F;
    telemetry.latitude_deg = 30.131798F;
    telemetry.satellites_used = 17U;
    telemetry.gps_altitude_m = 943.5F;
    telemetry.battery_decivolt = 247U;
    telemetry.relative_height_decimeter = 1234U;

    assert(core.request_command(cgtd070::Command::GreenBoxTracking));
    uint8_t frame[cgtd070::kControlFrameSize];
    core.build_next_tx_frame(frame, telemetry);

    assert(frame[0] == 0xFBU && frame[1] == 0x2CU);
    assert(frame[2] == 0x93U);
    assert(frame[3] == 0U && frame[4] == 0U);
    assert(frame[5] == 0U && frame[6] == 0U);
    assert(frame[7] == 0x2EU && frame[8] == 0xFBU); // -1234
    assert(frame[9] == 0x2EU && frame[10] == 0x16U); // 5678
    assert(frame[11] == 0x9FU && frame[12] == 0x8CU); // 35999
    assert(frame[13] == 26U && frame[14] == 8U && frame[15] == 12U);
    assert(frame[16] == 9U && frame[17] == 10U && frame[18] == 11U);
    assert(frame[19] == 42U);
    assert(near_value(get_f32(frame, 20U),
                      telemetry.longitude_deg, 0.0001F));
    assert(near_value(get_f32(frame, 24U),
                      telemetry.latitude_deg, 0.0001F));
    assert(frame[28] == 17U);
    assert(near_value(get_f32(frame, 29U), 943.5F));
    assert(frame[33] == 0xF7U && frame[34] == 0x00U);
    assert(frame[35] == 0xD2U && frame[36] == 0x04U);
    for (size_t i = 37U; i <= 41U; ++i) {
        assert(frame[i] == 0U);
    }
    assert(frame[42] == cgtd070::Core::control_checksum(frame));
    assert(frame[43] == 0xF0U);

    // The queued command is edge-triggered, but current telemetry remains in
    // every subsequent idle frame supplied by the application.
    core.build_next_tx_frame(frame, telemetry);
    assert(frame[2] == 0x00U);
    assert(frame[7] == 0x2EU && frame[8] == 0xFBU);
}

void test_json()
{
    uint8_t frame[cgtd070::kFeedbackFrameSize]{};
    frame[0] = 0xFCU;
    frame[1] = 0x2CU;
    frame[2] = 0xFFU;
    const char text[] = "{\"echo\":\"ok\"}";
    memcpy(frame + 3U, text, sizeof(text) - 1U);
    frame[62] = cgtd070::Core::feedback_checksum(frame);
    frame[63] = 0xF0U;

    cgtd070::Core core;
    core.ingest_rx(frame, sizeof(frame));
    cgtd070::Feedback output{};
    assert(core.take_feedback(output));
    assert(output.kind == cgtd070::FrameKind::Json);
    assert(strcmp(output.json, text) == 0);
}

} // namespace

int main()
{
    static_assert(sizeof(cgtd070::Core) <= 512U,
                  "core memory footprint unexpectedly increased");
    test_decode_fields();
    test_stream_chunks_noise_and_recovery();
    test_latest_frame_overrun();
    test_control_state_machine();
    test_control_flight_telemetry();
    test_json();
    printf("all cgtd070 core tests passed; Core size=%u bytes\n",
           static_cast<unsigned>(sizeof(cgtd070::Core)));
    return 0;
}
