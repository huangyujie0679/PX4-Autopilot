#pragma once

#include "cgtd070_protocol_types.hpp"

namespace cgtd070 {

class Core final {
public:
    Core() noexcept;

    Core(const Core &) = delete;
    Core &operator=(const Core &) = delete;

    // Accepts arbitrary stream chunks. The caller may provide one byte,
    // partial frames, multiple frames, or data containing noise.
    void ingest_rx(const uint8_t *data, size_t length) noexcept;

    // Returns the latest complete frame once. A newer frame overwrites an
    // unread older frame and increments feedback_overruns.
    bool take_feedback(Feedback &output) noexcept;

    // Queues one edge-triggered command. A pending non-idle command is not
    // overwritten; false asks the caller to retry on a later app cycle.
    bool request_command(Command command) noexcept;

    // Called by the application every 40 ms. It emits a queued command once,
    // consumes it, and emits an idle frame on subsequent calls.
    void build_next_tx_frame(uint8_t output[kControlFrameSize]) noexcept;

    // Same state machine, with protocol-ready flight-controller telemetry in
    // bytes 8..37. Missing source data is represented by zero-valued fields.
    void build_next_tx_frame(
        uint8_t output[kControlFrameSize],
        const FlightTelemetry &telemetry) noexcept;

    // Same frame with protocol-table-5 joystick control in bytes 38..42.
    // enabled=0 forces both rates to zero and exits manual-search mode.
    void build_next_tx_frame(
        uint8_t output[kControlFrameSize],
        const FlightTelemetry &telemetry,
        const JoystickControl &joystick) noexcept;

    const ParserStats &stats() const noexcept { return stats_; }
    DecodeError last_error() const noexcept { return last_error_; }
    size_t buffered_bytes() const noexcept { return rx_used_; }
    bool feedback_available() const noexcept { return feedback_available_; }

    void reset() noexcept;

    static uint8_t feedback_checksum(const uint8_t *frame) noexcept;
    static uint8_t control_checksum(const uint8_t *frame) noexcept;
    static DecodeError decode_frame(const uint8_t *data,
                                    size_t length,
                                    Feedback &output) noexcept;

private:
    void push_byte(uint8_t byte) noexcept;
    void resynchronize() noexcept;
    void accept_frame() noexcept;

    uint8_t rx_frame_[kFeedbackFrameSize];
    size_t rx_used_;
    Feedback latest_feedback_;
    ParserStats stats_;
    Command pending_command_;
    DecodeError last_error_;
    bool feedback_available_;
};

static_assert(kFeedbackFrameSize == 64U, "CGTD070 feedback frame must be 64 bytes");
static_assert(kControlFrameSize == 44U, "CGTD070 control frame must be 44 bytes");

} // namespace cgtd070
