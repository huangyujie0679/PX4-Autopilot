#include "cgtd070_app.hpp"
#include "cgtd070_rc_knob.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <errno.h>
#include <math.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint64_t kAttitudeMaxAgeUs = 500000U;
constexpr uint64_t kGpsMaxAgeUs = 2000000U;
constexpr uint64_t kBatteryMaxAgeUs = 2000000U;
constexpr uint64_t kGlobalPositionMaxAgeUs = 2000000U;
constexpr uint64_t kRcMaxAgeUs = 500000U;
constexpr uint64_t kRcDebounceUs = 100000U;
constexpr float kRcSwitchThreshold = 0.5F;
constexpr int16_t kRcAzimuthRateDps = 10;
constexpr int16_t kRcPitchRateDps = 10;
constexpr float kRadToDeg = 57.29577951308232F;
constexpr long kJoystickRateMinimum = -100L;
constexpr long kJoystickRateMaximum = 100L;
constexpr long kJoystickDurationMinimumMs = 40L;
constexpr long kJoystickDurationMaximumMs = 60000L;
constexpr uint8_t kDaysPerMonth[12] = {
    31U, 28U, 31U, 30U, 31U, 30U,
    31U, 31U, 30U, 31U, 30U, 31U
};

float constrain_float(float value, float lower, float upper) noexcept
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

int16_t read_i16_le(const uint8_t *data) noexcept
{
    const uint16_t value = static_cast<uint16_t>(data[0]) |
        static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
    return static_cast<int16_t>(value);
}

bool parse_long(const char *text, long lower, long upper, long &output) noexcept
{
    if (text == nullptr || text[0] == '\0') {
        return false;
    }

    errno = 0;
    char *end = nullptr;
    const long value = strtol(text, &end, 10);
    if (errno == ERANGE || end == text || end == nullptr || end[0] != '\0' ||
        value < lower || value > upper) {
        return false;
    }

    output = value;
    return true;
}

bool fresh(uint64_t sample, uint64_t now, uint64_t max_age) noexcept
{
    return sample != 0U && sample <= now && now - sample <= max_age;
}

int16_t degrees_to_cdeg(float degrees, float lower, float upper) noexcept
{
    if (!__builtin_isfinite(degrees)) {
        return 0;
    }
    const float limited = constrain_float(degrees, lower, upper);
    return static_cast<int16_t>(__builtin_lroundf(limited * 100.0F));
}

uint16_t nonnegative_decimals(float value, float scale, float maximum) noexcept
{
    if (!__builtin_isfinite(value) || value <= 0.0F) {
        return 0U;
    }
    const float limited = constrain_float(value, 0.0F, maximum);
    return static_cast<uint16_t>(__builtin_lroundf(limited * scale));
}

bool leap_year(uint32_t year) noexcept
{
    return (year % 4U == 0U && year % 100U != 0U) || year % 400U == 0U;
}

bool split_utc(uint64_t usec, cgtd070::FlightTelemetry &output) noexcept
{
    if (usec == 0U) {
        return false;
    }

    const uint64_t total_seconds = usec / 1000000ULL;
    uint32_t days = static_cast<uint32_t>(total_seconds / 86400ULL);
    uint32_t seconds_of_day = static_cast<uint32_t>(total_seconds % 86400ULL);
    uint32_t year = 1970U;

    while (year <= 2099U) {
        const uint32_t days_in_year = leap_year(year) ? 366U : 365U;
        if (days < days_in_year) {
            break;
        }
        days -= days_in_year;
        ++year;
    }
    if (year < 2000U || year > 2099U) {
        return false;
    }

    uint32_t month = 0U;
    while (month < 12U) {
        uint32_t month_days = kDaysPerMonth[month];
        if (month == 1U && leap_year(year)) {
            month_days = 29U;
        }
        if (days < month_days) {
            break;
        }
        days -= month_days;
        ++month;
    }
    if (month >= 12U) {
        return false;
    }

    output.gps_year_since_2000 = static_cast<uint8_t>(year - 2000U);
    output.gps_month = static_cast<uint8_t>(month + 1U);
    output.gps_day = static_cast<uint8_t>(days + 1U);
    output.gps_hour = static_cast<uint8_t>(seconds_of_day / 3600U);
    seconds_of_day %= 3600U;
    output.gps_minute = static_cast<uint8_t>(seconds_of_day / 60U);
    output.gps_second = static_cast<uint8_t>(seconds_of_day % 60U);
    output.gps_centisecond = static_cast<uint8_t>((usec / 10000ULL) % 100ULL);
    return true;
}

void copy_text(char *destination, size_t capacity, const char *source) noexcept
{
    if (destination == nullptr || capacity == 0U) {
        return;
    }

    destination[0] = '\0';
    if (source == nullptr) {
        return;
    }

    size_t i = 0U;
    while (i + 1U < capacity && source[i] != '\0') {
        destination[i] = source[i];
        ++i;
    }
    destination[i] = '\0';
}

const char *open_stage_text(cgtd070::SerialOpenStage stage) noexcept
{
    switch (stage) {
    case cgtd070::SerialOpenStage::OpenDevice:
        return "open";
    case cgtd070::SerialOpenStage::GetAttributes:
        return "tcgetattr";
    case cgtd070::SerialOpenStage::SetAttributes:
        return "tcsetattr";
    case cgtd070::SerialOpenStage::None:
    default:
        return "none";
    }
}

} // namespace

Cgtd070App::Cgtd070App(const Config &config) noexcept :
    config_(config),
    core_(),
    serial_(),
    rc_switch_stable_(0),
    rc_switch_candidate_(0),
    rc_switch_candidate_since_(0U),
    rc_center_stable_(0U),
    rc_center_candidate_(0U),
    rc_center_candidate_since_(0U),
    rc_state_initialized_(0U),
    rc_input_valid_(0U),
    rc_aux2_milli_(0),
    rc_aux3_milli_(0),
    rc_switch_status_(0),
    rc_center_status_(0U),
    rc_commands_(0U),
    rc_command_busy_(0U),
    rc_azimuth_knob_state_(0),
    rc_pitch_knob_state_(0),
    rc_manual_armed_(0U),
    rc_joystick_control_{},
    rc_aux4_milli_(0),
    rc_aux5_milli_(0),
    rc_azimuth_knob_status_(0),
    rc_pitch_knob_status_(0),
    rc_manual_armed_status_(0U),
    tx_buffer_{},
    tx_offset_(cgtd070::kControlFrameSize),
    next_tx_us_(0U),
    joystick_control_{},
    joystick_end_us_(0U),
    trace_joystick_enabled_(0U),
    trace_joystick_azimuth_rate_(0),
    trace_joystick_pitch_rate_(0),
    tx_frames_(0U),
    rx_frames_(0U),
    io_errors_(0U),
    poll_errors_(0U),
    poll_error_(0),
    missed_tx_slots_(0U),
    parser_bytes_(0U),
    parser_ok_(0U),
    parser_checksum_(0U),
    parser_framing_(0U),
    parser_overruns_(0U),
    feedback_counter_(0U),
    feedback_status_primary_(0U),
    feedback_status_secondary_(0U),
    feedback_servo_state_(0U),
    command_mailbox_(0U),
    joystick_azimuth_mailbox_(0),
    joystick_pitch_mailbox_(0),
    joystick_duration_ms_mailbox_(0U),
    joystick_mailbox_state_(0U),
    joystick_active_status_(0U),
    joystick_azimuth_status_(0),
    joystick_pitch_status_(0),
    joystick_remaining_ms_status_(0U),
    joystick_source_status_(0U),
    have_feedback_(0U),
    trace_enabled_(0U)
{
}

Cgtd070App::~Cgtd070App()
{
    // ModuleBase deletes the object from run_trampoline(), so this close runs
    // in the same Task context that opened and used the descriptor.
    serial_.close();
}

int Cgtd070App::task_spawn(int argc, char *argv[])
{
    (void)argc;
    _task_id = px4_task_spawn_cmd("cgtd070",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  kTaskStackSize,
                                  reinterpret_cast<px4_main_t>(&run_trampoline),
                                  reinterpret_cast<char *const *>(argv));

    if (_task_id < 0) {
        const int error = errno;
        _task_id = -1;
        return -error;
    }

    // Report an immediate instantiate/open failure to the `start` command.
    return wait_until_running(1000);
}

Cgtd070App *Cgtd070App::instantiate(int argc, char *argv[])
{
    Config config{};
    if (!parse_config(argc, argv, config)) {
        return nullptr;
    }

    Cgtd070App *instance = new Cgtd070App(config);
    if (instance == nullptr) {
        PX4_ERR("alloc failed");
        return nullptr;
    }

    // instantiate() is executed by run_trampoline() inside the new Task.
    // Opening here guarantees open/read/write/close share one task group.
    if (!instance->open_serial()) {
        delete instance;
        return nullptr;
    }

    return instance;
}

bool Cgtd070App::open_serial() noexcept
{
    if (serial_.open(config_.serial_device)) {
        PX4_INFO("independent Task opened %s fd=%d",
                 config_.serial_device, serial_.fd());
        return true;
    }

    PX4_ERR("serial %s failed at %s: errno=%d",
            config_.serial_device,
            open_stage_text(serial_.open_stage()),
            serial_.open_error());
    return false;
}

void Cgtd070App::run()
{
    next_tx_us_ = hrt_absolute_time();

    while (!should_exit()) {
        uint64_t now = hrt_absolute_time();
        update_flight_sources();
        update_rc_control(now);
        prepare_tx_if_due(now);

        px4_pollfd_struct_t poll_fd{};
        poll_fd.fd = serial_.fd();
        poll_fd.events = POLLIN;
        if (tx_offset_ < cgtd070::kControlFrameSize) {
            poll_fd.events |= POLLOUT;
        }

        const int poll_result = px4_poll(&poll_fd, 1U, poll_timeout_ms(now));
        if (poll_result < 0 && errno != EINTR) {
            __atomic_store_n(&poll_error_, errno, __ATOMIC_RELEASE);
            __atomic_add_fetch(&poll_errors_, 1U, __ATOMIC_RELAXED);
            __atomic_add_fetch(&io_errors_, 1U, __ATOMIC_RELAXED);
            px4_usleep(10000U);
        }

        // read_available() and write_some() are non-blocking. Calling them
        // after timeout as well handles UART drivers with limited poll flags.
        service_rx();
        service_tx();
    }

    serial_.close();
}

void Cgtd070App::service_rx() noexcept
{
    uint8_t rx_buffer[128];

    // Bound each pass so continuous RX cannot starve TX timing or stop.
    for (unsigned attempt = 0U; attempt < 8U; ++attempt) {
        const int received = serial_.read_available(rx_buffer, sizeof(rx_buffer));
        if (received < 0) {
            __atomic_add_fetch(&io_errors_, 1U, __ATOMIC_RELAXED);
            break;
        }
        if (received == 0) {
            break;
        }

        core_.ingest_rx(rx_buffer, static_cast<size_t>(received));
        update_parser_snapshot();

        cgtd070::Feedback feedback;
        if (core_.take_feedback(feedback)) {
            process_feedback(feedback);
        }
    }
}

void Cgtd070App::prepare_tx_if_due(uint64_t now) noexcept
{
    if (now < next_tx_us_) {
        return;
    }

    const uint64_t elapsed = now - next_tx_us_;
    const uint32_t due_slots =
        static_cast<uint32_t>(elapsed / kTxIntervalUs) + 1U;
    next_tx_us_ += static_cast<uint64_t>(due_slots) * kTxIntervalUs;

    if (tx_offset_ < cgtd070::kControlFrameSize) {
        // Never interleave a new protocol frame with an unfinished one.
        __atomic_add_fetch(&missed_tx_slots_, due_slots, __ATOMIC_RELAXED);
        return;
    }

    if (due_slots > 1U) {
        __atomic_add_fetch(&missed_tx_slots_, due_slots - 1U, __ATOMIC_RELAXED);
    }

    const uint8_t joystick_request = __atomic_load_n(
        &joystick_mailbox_state_, __ATOMIC_ACQUIRE);
    if (joystick_request == kJoystickRequestMove ||
        joystick_request == kJoystickRequestStop) {
        if (joystick_request == kJoystickRequestMove) {
            const int16_t azimuth_rate_dps = static_cast<int16_t>(
                __atomic_load_n(&joystick_azimuth_mailbox_, __ATOMIC_ACQUIRE));
            const int16_t pitch_rate_dps = static_cast<int16_t>(
                __atomic_load_n(&joystick_pitch_mailbox_, __ATOMIC_ACQUIRE));
            const uint32_t duration_ms = __atomic_load_n(
                &joystick_duration_ms_mailbox_, __ATOMIC_ACQUIRE);
            (void)__atomic_exchange_n(&joystick_mailbox_state_, 0U,
                                      __ATOMIC_ACQ_REL);
            joystick_control_.enabled = 1U;
            joystick_control_.azimuth_rate_dps = azimuth_rate_dps;
            joystick_control_.pitch_rate_dps = pitch_rate_dps;
            joystick_end_us_ = now + static_cast<uint64_t>(duration_ms) * 1000ULL;
            disarm_rc_manual();
        } else {
            (void)__atomic_exchange_n(&joystick_mailbox_state_, 0U,
                                      __ATOMIC_ACQ_REL);
            joystick_control_ = cgtd070::JoystickControl{};
            joystick_end_us_ = 0U;
            disarm_rc_manual();
        }
    }

    const uint8_t requested = __atomic_exchange_n(
        &command_mailbox_, 0U, __ATOMIC_ACQ_REL);
    if (requested != 0U) {
        // Protocol V2.12 requires joystick byte 38 to be 00H before another
        // servo-platform function can take control. The regular command frame
        // therefore also terminates any active timed manual movement.
        joystick_control_ = cgtd070::JoystickControl{};
        joystick_end_us_ = 0U;
        disarm_rc_manual();
        (void)core_.request_command(static_cast<cgtd070::Command>(requested));
    }

    if (joystick_control_.enabled != 0U && now >= joystick_end_us_) {
        joystick_control_ = cgtd070::JoystickControl{};
        joystick_end_us_ = 0U;
    }

    cgtd070::JoystickControl tx_joystick = joystick_control_;
    uint32_t joystick_source = tx_joystick.enabled != 0U ? 1U : 0U;
    if (rc_joystick_control_.enabled != 0U) {
        tx_joystick = rc_joystick_control_;
        joystick_source = 2U;
    }

    uint32_t remaining_ms = 0U;
    if (joystick_source == 1U && joystick_end_us_ > now) {
        remaining_ms = static_cast<uint32_t>(
            (joystick_end_us_ - now + 999ULL) / 1000ULL);
    }
    __atomic_store_n(&joystick_active_status_,
                     tx_joystick.enabled != 0U ? 1U : 0U,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&joystick_azimuth_status_,
                     static_cast<int32_t>(tx_joystick.azimuth_rate_dps),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&joystick_pitch_status_,
                     static_cast<int32_t>(tx_joystick.pitch_rate_dps),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&joystick_remaining_ms_status_, remaining_ms,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&joystick_source_status_, joystick_source,
                     __ATOMIC_RELEASE);

    // The host-tested protocol packer emits one requested command frame or
    // an idle 00H frame and inserts this cycle's zero-safe telemetry snapshot.
    const cgtd070::FlightTelemetry telemetry = make_flight_telemetry(now);
    core_.build_next_tx_frame(tx_buffer_, telemetry, tx_joystick);
    tx_offset_ = 0U;
}

void Cgtd070App::service_tx() noexcept
{
    if (tx_offset_ >= cgtd070::kControlFrameSize) {
        return;
    }

    const size_t remaining = cgtd070::kControlFrameSize - tx_offset_;
    const int written = serial_.write_some(tx_buffer_ + tx_offset_, remaining);
    if (written < 0) {
        __atomic_add_fetch(&io_errors_, 1U, __ATOMIC_RELAXED);

        // A hard error after a partial write leaves an incomplete frame on the
        // wire. Drop its remainder; the receiver can resynchronise at the next
        // FB 2C header. EAGAIN/EINTR return zero and keep the frame pending.
        tx_offset_ = cgtd070::kControlFrameSize;
        return;
    }

    if (written == 0) {
        return;
    }

    tx_offset_ += static_cast<size_t>(written);
    if (tx_offset_ >= cgtd070::kControlFrameSize) {
        __atomic_add_fetch(&tx_frames_, 1U, __ATOMIC_RELAXED);
        trace_completed_command_frame();
    }
}

void Cgtd070App::trace_completed_command_frame() noexcept
{
    const uint8_t command = tx_buffer_[2];
    const uint8_t joystick_enabled = tx_buffer_[37] == 0x70U ? 1U : 0U;
    const int16_t joystick_azimuth_rate = read_i16_le(tx_buffer_ + 38U);
    const int16_t joystick_pitch_rate = read_i16_le(tx_buffer_ + 40U);

    if (__atomic_load_n(&trace_enabled_, __ATOMIC_ACQUIRE) == 0U) {
        trace_joystick_enabled_ = 0U;
        trace_joystick_azimuth_rate_ = 0;
        trace_joystick_pitch_rate_ = 0;
        return;
    }

    const bool joystick_changed =
        joystick_enabled != trace_joystick_enabled_ ||
        (joystick_enabled != 0U &&
         (joystick_azimuth_rate != trace_joystick_azimuth_rate_ ||
          joystick_pitch_rate != trace_joystick_pitch_rate_));

    // Idle frames continue at 25 Hz. For manual motion, trace only its first
    // enabled frame, a changed-rate frame, and the final disabled frame.
    if (command == static_cast<uint8_t>(cgtd070::Command::Idle) &&
        !joystick_changed) {
        return;
    }

    trace_joystick_enabled_ = joystick_enabled;
    trace_joystick_azimuth_rate_ = joystick_azimuth_rate;
    trace_joystick_pitch_rate_ = joystick_pitch_rate;

    const char *name = "unknown";
    switch (static_cast<cgtd070::Command>(command)) {
    case cgtd070::Command::GreenBoxTracking:
        name = "track";
        break;
    case cgtd070::Command::Center:
        name = "center";
        break;
    case cgtd070::Command::YawFollowAircraft:
        name = "yaw_follow";
        break;
    case cgtd070::Command::CancelTracking:
        name = "cancel";
        break;
    case cgtd070::Command::Idle:
    default:
        break;
    }

    if (command == static_cast<uint8_t>(cgtd070::Command::Idle)) {
        PX4_INFO("TX complete joystick=%s azimuth=%d deg/s pitch=%d deg/s, 44-byte frame:",
                 joystick_enabled != 0U ? "move" : "stop",
                 static_cast<int>(joystick_azimuth_rate),
                 static_cast<int>(joystick_pitch_rate));
    } else {
        PX4_INFO("TX complete command=%s (0x%02x), 44-byte frame:",
                 name, static_cast<unsigned>(command));
    }
    PX4_INFO("TX[00..10] %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx",
             tx_buffer_[0], tx_buffer_[1], tx_buffer_[2], tx_buffer_[3],
             tx_buffer_[4], tx_buffer_[5], tx_buffer_[6], tx_buffer_[7],
             tx_buffer_[8], tx_buffer_[9], tx_buffer_[10]);
    PX4_INFO("TX[11..21] %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx",
             tx_buffer_[11], tx_buffer_[12], tx_buffer_[13], tx_buffer_[14],
             tx_buffer_[15], tx_buffer_[16], tx_buffer_[17], tx_buffer_[18],
             tx_buffer_[19], tx_buffer_[20], tx_buffer_[21]);
    PX4_INFO("TX[22..32] %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx",
             tx_buffer_[22], tx_buffer_[23], tx_buffer_[24], tx_buffer_[25],
             tx_buffer_[26], tx_buffer_[27], tx_buffer_[28], tx_buffer_[29],
             tx_buffer_[30], tx_buffer_[31], tx_buffer_[32]);
    PX4_INFO("TX[33..43] %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx %02hhx",
             tx_buffer_[33], tx_buffer_[34], tx_buffer_[35], tx_buffer_[36],
             tx_buffer_[37], tx_buffer_[38], tx_buffer_[39], tx_buffer_[40],
             tx_buffer_[41], tx_buffer_[42], tx_buffer_[43]);
}

int Cgtd070App::poll_timeout_ms(uint64_t now) const noexcept
{
    if (now >= next_tx_us_) {
        return 0;
    }

    const uint64_t remaining_us = next_tx_us_ - now;
    uint64_t timeout_ms = (remaining_us + 999U) / 1000U;
    if (timeout_ms > static_cast<uint64_t>(kMaxPollTimeoutMs)) {
        timeout_ms = static_cast<uint64_t>(kMaxPollTimeoutMs);
    }
    return static_cast<int>(timeout_ms);
}

bool Cgtd070App::queue_command(cgtd070::Command command) noexcept
{
    uint8_t expected = 0U;
    const uint8_t requested = static_cast<uint8_t>(command);
    return __atomic_compare_exchange_n(
        &command_mailbox_, &expected, requested, false,
        __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

bool Cgtd070App::queue_joystick_request(uint8_t request,
                                        int16_t azimuth_rate_dps,
                                        int16_t pitch_rate_dps,
                                        uint32_t duration_ms) noexcept
{
    uint8_t expected = 0U;
    if (!__atomic_compare_exchange_n(
            &joystick_mailbox_state_, &expected, kJoystickMailboxWriting,
            false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        return false;
    }

    __atomic_store_n(&joystick_azimuth_mailbox_,
                     static_cast<int32_t>(azimuth_rate_dps),
                     __ATOMIC_RELAXED);
    __atomic_store_n(&joystick_pitch_mailbox_,
                     static_cast<int32_t>(pitch_rate_dps),
                     __ATOMIC_RELAXED);
    __atomic_store_n(&joystick_duration_ms_mailbox_, duration_ms,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&joystick_mailbox_state_, request, __ATOMIC_RELEASE);
    return true;
}

void Cgtd070App::update_flight_sources() noexcept
{
    // update() is non-blocking. Each topic keeps its latest snapshot; a
    // missing publisher therefore cannot delay the 25 Hz protocol schedule.
    (void)attitude_sub_.update(&attitude_);
    (void)gps_sub_.update(&gps_);
    (void)battery_sub_.update(&battery_);
    (void)global_position_sub_.update(&global_position_);
    (void)home_position_sub_.update(&home_position_);
    (void)manual_control_sub_.update(&manual_control_);
}

void Cgtd070App::reset_rc_state() noexcept
{
    rc_state_initialized_ = 0U;
    rc_switch_candidate_since_ = 0U;
    rc_center_candidate_since_ = 0U;
    disarm_rc_manual();
}

void Cgtd070App::disarm_rc_manual() noexcept
{
    rc_azimuth_knob_state_ = 0;
    rc_pitch_knob_state_ = 0;
    rc_manual_armed_ = 0U;
    rc_joystick_control_ = cgtd070::JoystickControl{};
    __atomic_store_n(&rc_azimuth_knob_status_, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&rc_pitch_knob_status_, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&rc_manual_armed_status_, 0U, __ATOMIC_RELEASE);
}

void Cgtd070App::update_rc_control(uint64_t now) noexcept
{
    // Only a fresh, valid physical RC source may command the gimbal. AUX1 is
    // deliberately not read so it remains available to other PX4 functions.
    // MAVLink, joystick, stale data, unmapped AUX inputs, and RC loss are
    // ignored.
    const bool valid = manual_control_.valid
                       && manual_control_.data_source
                              == manual_control_setpoint_s::SOURCE_RC
                       && fresh(manual_control_.timestamp, now, kRcMaxAgeUs)
                       && __builtin_isfinite(manual_control_.aux2)
                       && __builtin_isfinite(manual_control_.aux3)
                       && __builtin_isfinite(manual_control_.aux4)
                       && __builtin_isfinite(manual_control_.aux5);

    __atomic_store_n(&rc_input_valid_, valid ? 1U : 0U, __ATOMIC_RELEASE);
    if (!valid) {
        __atomic_store_n(&rc_aux2_milli_, 0, __ATOMIC_RELEASE);
        __atomic_store_n(&rc_aux3_milli_, 0, __ATOMIC_RELEASE);
        __atomic_store_n(&rc_aux4_milli_, 0, __ATOMIC_RELEASE);
        __atomic_store_n(&rc_aux5_milli_, 0, __ATOMIC_RELEASE);
        reset_rc_state();
        return;
    }

    __atomic_store_n(&rc_aux2_milli_,
                     static_cast<int32_t>(manual_control_.aux2 * 1000.0F),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&rc_aux3_milli_,
                     static_cast<int32_t>(manual_control_.aux3 * 1000.0F),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&rc_aux4_milli_,
                     static_cast<int32_t>(manual_control_.aux4 * 1000.0F),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&rc_aux5_milli_,
                     static_cast<int32_t>(manual_control_.aux5 * 1000.0F),
                     __ATOMIC_RELEASE);

    const int8_t switch_position =
        manual_control_.aux2 > kRcSwitchThreshold ? 1
        : (manual_control_.aux2 < -kRcSwitchThreshold ? -1 : 0);
    const uint8_t center_pressed =
        manual_control_.aux3 > kRcSwitchThreshold ? 1U : 0U;

    // On startup or after RC recovery, learn current positions without
    // commanding. The pilot must make a new deliberate control movement.
    if (rc_state_initialized_ == 0U) {
        rc_switch_stable_ = switch_position;
        rc_switch_candidate_ = switch_position;
        rc_switch_candidate_since_ = now;
        rc_center_stable_ = center_pressed;
        rc_center_candidate_ = center_pressed;
        rc_center_candidate_since_ = now;
        rc_state_initialized_ = 1U;
        __atomic_store_n(&rc_switch_status_,
                         static_cast<int32_t>(rc_switch_stable_),
                         __ATOMIC_RELEASE);
        __atomic_store_n(&rc_center_status_,
                         static_cast<uint32_t>(rc_center_stable_),
                         __ATOMIC_RELEASE);
        return;
    }

    if (switch_position != rc_switch_candidate_) {
        rc_switch_candidate_ = switch_position;
        rc_switch_candidate_since_ = now;

    } else if (switch_position != rc_switch_stable_
               && now - rc_switch_candidate_since_ >= kRcDebounceUs) {
        cgtd070::Command command = cgtd070::Command::Idle;

        if (switch_position > 0) {
            command = cgtd070::Command::GreenBoxTracking;

        } else if (switch_position < 0) {
            command = cgtd070::Command::CancelTracking;
        }

        // Middle is command-free. For low/high, only commit the new state
        // after its command enters the mailbox; a busy mailbox is retried.
        bool committed = command == cgtd070::Command::Idle;
        if (!committed && queue_command(command)) {
            __atomic_add_fetch(&rc_commands_, 1U, __ATOMIC_RELAXED);
            committed = true;

        } else if (!committed) {
            __atomic_add_fetch(&rc_command_busy_, 1U, __ATOMIC_RELAXED);
        }

        if (committed) {
            rc_switch_stable_ = switch_position;
            __atomic_store_n(&rc_switch_status_,
                             static_cast<int32_t>(rc_switch_stable_),
                             __ATOMIC_RELEASE);
        }
    }

    if (center_pressed != rc_center_candidate_) {
        rc_center_candidate_ = center_pressed;
        rc_center_candidate_since_ = now;

    } else if (center_pressed != rc_center_stable_
               && now - rc_center_candidate_since_ >= kRcDebounceUs) {
        // Only the transition into AUX3 high queues one 71H frame. Holding
        // high or returning to middle/low cannot repeat the command.
        bool committed = center_pressed == 0U;
        if (!committed && queue_command(cgtd070::Command::Center)) {
            __atomic_add_fetch(&rc_commands_, 1U, __ATOMIC_RELAXED);
            committed = true;

        } else if (!committed) {
            __atomic_add_fetch(&rc_command_busy_, 1U, __ATOMIC_RELAXED);
        }

        if (committed) {
            rc_center_stable_ = center_pressed;
            __atomic_store_n(&rc_center_status_,
                             static_cast<uint32_t>(rc_center_stable_),
                             __ATOMIC_RELEASE);
        }
    }

    const int8_t azimuth_position = cgtd070::rc_knob_position(
        manual_control_.aux4, rc_azimuth_knob_state_);
    const int8_t pitch_position = cgtd070::rc_knob_position(
        manual_control_.aux5, rc_pitch_knob_state_);

    if (rc_manual_armed_ == 0U) {
        rc_joystick_control_ = cgtd070::JoystickControl{};
        rc_azimuth_knob_state_ = azimuth_position;
        rc_pitch_knob_state_ = pitch_position;

        // After startup, RC recovery, an explicit stop, or another gimbal
        // command, both knobs must return to their middle region before
        // endpoint movement can be armed again.
        if (azimuth_position == 0 && pitch_position == 0) {
            rc_manual_armed_ = 1U;
        }
    } else {
        rc_azimuth_knob_state_ = azimuth_position;
        rc_pitch_knob_state_ = pitch_position;
        rc_joystick_control_.azimuth_rate_dps = static_cast<int16_t>(
            static_cast<int16_t>(azimuth_position) * kRcAzimuthRateDps);
        rc_joystick_control_.pitch_rate_dps = static_cast<int16_t>(
            static_cast<int16_t>(pitch_position) * kRcPitchRateDps);
        rc_joystick_control_.enabled =
            azimuth_position != 0 || pitch_position != 0 ? 1U : 0U;
    }

    __atomic_store_n(&rc_azimuth_knob_status_,
                     static_cast<int32_t>(rc_azimuth_knob_state_),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&rc_pitch_knob_status_,
                     static_cast<int32_t>(rc_pitch_knob_state_),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&rc_manual_armed_status_,
                     static_cast<uint32_t>(rc_manual_armed_),
                     __ATOMIC_RELEASE);
}

cgtd070::FlightTelemetry Cgtd070App::make_flight_telemetry(
    uint64_t now) const noexcept
{
    // Protocol V2.12 explicitly requires zero when flight data is absent.
    // Start from zero every 40 ms, then overwrite only valid, fresh groups.
    cgtd070::FlightTelemetry output{};

    if (fresh(attitude_.timestamp, now, kAttitudeMaxAgeUs)) {
        const float q0 = attitude_.q[0];
        const float q1 = attitude_.q[1];
        const float q2 = attitude_.q[2];
        const float q3 = attitude_.q[3];
        const float norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;

        if (__builtin_isfinite(norm_sq) && norm_sq > 0.25F) {
            const float inv_norm = 1.0F / __builtin_sqrtf(norm_sq);
            const float w = q0 * inv_norm;
            const float x = q1 * inv_norm;
            const float y = q2 * inv_norm;
            const float z = q3 * inv_norm;
            const float roll = __builtin_atan2f(
                2.0F * (w * x + y * z),
                1.0F - 2.0F * (x * x + y * y));
            const float pitch_sin = constrain_float(
                2.0F * (w * y - z * x), -1.0F, 1.0F);
            const float pitch = __builtin_asinf(pitch_sin);
            float heading_deg = __builtin_atan2f(
                                    2.0F * (w * z + x * y),
                                    1.0F - 2.0F * (y * y + z * z))
                                * kRadToDeg;
            if (heading_deg < 0.0F) {
                heading_deg += 360.0F;
            }

            output.pitch_cdeg = degrees_to_cdeg(
                pitch * kRadToDeg, -90.0F, 90.0F);
            output.roll_cdeg = degrees_to_cdeg(
                roll * kRadToDeg, -90.0F, 90.0F);
            uint32_t heading = static_cast<uint32_t>(
                __builtin_lroundf(heading_deg * 100.0F));
            if (heading >= 36000U) {
                heading = 0U;
            }
            output.heading_cdeg = static_cast<uint16_t>(heading);
        }
    }

    if (fresh(gps_.timestamp, now, kGpsMaxAgeUs)
        && gps_.fix_type >= sensor_gps_s::FIX_TYPE_2D) {
        if (__builtin_isfinite(gps_.longitude_deg)
            && gps_.longitude_deg >= -180.0
            && gps_.longitude_deg <= 180.0) {
            output.longitude_deg = static_cast<float>(gps_.longitude_deg);
        }
        if (__builtin_isfinite(gps_.latitude_deg)
            && gps_.latitude_deg >= -90.0
            && gps_.latitude_deg <= 90.0) {
            output.latitude_deg = static_cast<float>(gps_.latitude_deg);
        }
        output.satellites_used = gps_.satellites_used > 24U
                                 ? 24U : gps_.satellites_used;
        if (gps_.fix_type >= sensor_gps_s::FIX_TYPE_3D
            && __builtin_isfinite(gps_.altitude_msl_m)) {
            output.gps_altitude_m = constrain_float(
                static_cast<float>(gps_.altitude_msl_m), -500.0F, 6000.0F);
        }

        (void)split_utc(gps_.time_utc_usec, output);
    }

    if (fresh(battery_.timestamp, now, kBatteryMaxAgeUs)
        && battery_.connected) {
        output.battery_decivolt = nonnegative_decimals(
            battery_.voltage_v, 10.0F, 6553.5F);
    }

    if (fresh(global_position_.timestamp, now, kGlobalPositionMaxAgeUs)
        && global_position_.alt_valid && home_position_.valid_alt
        && __builtin_isfinite(global_position_.alt)
        && __builtin_isfinite(home_position_.alt)) {
        output.relative_height_decimeter = nonnegative_decimals(
            global_position_.alt - home_position_.alt, 10.0F, 6000.0F);
    }

    return output;
}

void Cgtd070App::process_feedback(const cgtd070::Feedback &feedback) noexcept
{
    __atomic_add_fetch(&rx_frames_, 1U, __ATOMIC_RELAXED);

    // All feedback kinds are decoded by the transport-independent core, but
    // guidance receives only the seven requested fields from telemetry frames.
    if (feedback.kind != cgtd070::FrameKind::Telemetry) {
        return;
    }

    cgtd070_status_s message{};
    message.timestamp = hrt_absolute_time();

    const cgtd070::Telemetry &t = feedback.telemetry;
    message.los_az_rate = t.target_los_azimuth_rate_dps;
    message.los_el_rate = t.target_los_pitch_rate_dps;
    // The requested field name gimbal_roll carries the protocol's gimbal
    // azimuth/frame angle; gimbal_pitch carries the pitch frame angle.
    message.gimbal_roll = t.gimbal_azimuth_deg;
    message.gimbal_pitch = t.gimbal_pitch_deg;
    message.los_azimuth = t.miss_azimuth_deg;
    message.los_elevation = t.miss_pitch_deg;
    message.target_valid = t.servo_state == 0x07U;

    status_pub_.publish(message);
    __atomic_store_n(&feedback_counter_,
                     static_cast<uint32_t>(t.frame_counter),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&feedback_status_primary_,
                     static_cast<uint32_t>(t.status_primary),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&feedback_status_secondary_,
                     static_cast<uint32_t>(t.status_secondary),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&feedback_servo_state_,
                     static_cast<uint32_t>(t.servo_state),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&have_feedback_, 1U, __ATOMIC_RELEASE);
}

void Cgtd070App::update_parser_snapshot() noexcept
{
    const cgtd070::ParserStats &stats = core_.stats();
    __atomic_store_n(&parser_bytes_, stats.bytes_received, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_ok_, stats.frames_ok, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_checksum_, stats.checksum_errors, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_framing_, stats.framing_errors, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_overruns_, stats.feedback_overruns, __ATOMIC_RELEASE);
}

int Cgtd070App::print_status()
{
    PX4_INFO("mode: independent Task, serial=%s fd=%d",
             config_.serial_device, serial_.fd());
    PX4_INFO("TX=%lu RX=%lu IO errors=%lu missed TX slots=%lu",
             static_cast<unsigned long>(
                 __atomic_load_n(&tx_frames_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&rx_frames_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&io_errors_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&missed_tx_slots_, __ATOMIC_ACQUIRE)));
    PX4_INFO("serial open=%d read=%d write=%d read errors=%lu write errors=%lu EAGAIN=%lu",
             serial_.open_error(),
             serial_.read_error(),
             serial_.write_error(),
             static_cast<unsigned long>(serial_.read_errors()),
             static_cast<unsigned long>(serial_.write_errors()),
             static_cast<unsigned long>(serial_.write_would_block()));
    PX4_INFO("poll errno=%d errors=%lu",
             __atomic_load_n(&poll_error_, __ATOMIC_ACQUIRE),
             static_cast<unsigned long>(
                 __atomic_load_n(&poll_errors_, __ATOMIC_ACQUIRE)));
    PX4_INFO("parser bytes=%lu ok=%lu checksum=%lu framing=%lu overruns=%lu",
             static_cast<unsigned long>(
                 __atomic_load_n(&parser_bytes_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&parser_ok_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&parser_checksum_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&parser_framing_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&parser_overruns_, __ATOMIC_ACQUIRE)));
    PX4_INFO("RC AUX2x1000=%ld AUX3x1000=%ld valid=%u switch=%ld center=%lu commands=%lu busy=%lu",
             static_cast<long>(
                 __atomic_load_n(&rc_aux2_milli_, __ATOMIC_ACQUIRE)),
             static_cast<long>(
                 __atomic_load_n(&rc_aux3_milli_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned>(
                 __atomic_load_n(&rc_input_valid_, __ATOMIC_ACQUIRE)),
             static_cast<long>(
                 __atomic_load_n(&rc_switch_status_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&rc_center_status_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&rc_commands_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&rc_command_busy_, __ATOMIC_ACQUIRE)));
    PX4_INFO("RC knobs AUX4x1000=%ld AUX5x1000=%ld azimuth_pos=%ld pitch_pos=%ld armed=%lu",
             static_cast<long>(
                 __atomic_load_n(&rc_aux4_milli_, __ATOMIC_ACQUIRE)),
             static_cast<long>(
                 __atomic_load_n(&rc_aux5_milli_, __ATOMIC_ACQUIRE)),
             static_cast<long>(
                 __atomic_load_n(&rc_azimuth_knob_status_, __ATOMIC_ACQUIRE)),
             static_cast<long>(
                 __atomic_load_n(&rc_pitch_knob_status_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&rc_manual_armed_status_, __ATOMIC_ACQUIRE)));
    const uint32_t joystick_source = __atomic_load_n(
        &joystick_source_status_, __ATOMIC_ACQUIRE);
    const char *joystick_source_name = joystick_source == 2U ? "RC"
                                       : (joystick_source == 1U ? "timed" : "none");
    PX4_INFO("manual move source=%s active=%lu azimuth=%ld deg/s pitch=%ld deg/s remaining=%lu ms mailbox=%u",
             joystick_source_name,
             static_cast<unsigned long>(
                 __atomic_load_n(&joystick_active_status_, __ATOMIC_ACQUIRE)),
             static_cast<long>(
                 __atomic_load_n(&joystick_azimuth_status_, __ATOMIC_ACQUIRE)),
             static_cast<long>(
                 __atomic_load_n(&joystick_pitch_status_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned long>(
                 __atomic_load_n(&joystick_remaining_ms_status_, __ATOMIC_ACQUIRE)),
             static_cast<unsigned>(
                 __atomic_load_n(&joystick_mailbox_state_, __ATOMIC_ACQUIRE)));
    PX4_INFO("command-frame trace=%s",
             __atomic_load_n(&trace_enabled_, __ATOMIC_ACQUIRE) != 0U
                 ? "on" : "off");

    if (__atomic_load_n(&have_feedback_, __ATOMIC_ACQUIRE) != 0U) {
        PX4_INFO("feedback counter=%lu status1=0x%04lx status2=0x%04lx servo=0x%02lx",
                 static_cast<unsigned long>(
                     __atomic_load_n(&feedback_counter_, __ATOMIC_ACQUIRE)),
                 static_cast<unsigned long>(
                     __atomic_load_n(&feedback_status_primary_, __ATOMIC_ACQUIRE)),
                 static_cast<unsigned long>(
                     __atomic_load_n(&feedback_status_secondary_, __ATOMIC_ACQUIRE)),
                 static_cast<unsigned long>(
                     __atomic_load_n(&feedback_servo_state_, __ATOMIC_ACQUIRE)));
    }

    return PX4_OK;
}

int Cgtd070App::custom_command(int argc, char *argv[])
{
    if (argc < 1 || argv == nullptr || argv[0] == nullptr) {
        return print_usage("missing command");
    }

    Cgtd070App *instance = get_instance();
    if (instance == nullptr) {
        PX4_ERR("module is not running");
        return PX4_ERROR;
    }

    if (strcmp(argv[0], "trace") == 0) {
        if (argc < 2 || argv[1] == nullptr) {
            return print_usage("trace requires on or off");
        }

        uint8_t enabled = 0U;
        if (strcmp(argv[1], "on") == 0) {
            enabled = 1U;
        } else if (strcmp(argv[1], "off") != 0) {
            return print_usage("trace requires on or off");
        }

        __atomic_store_n(&instance->trace_enabled_, enabled, __ATOMIC_RELEASE);
        PX4_INFO("command-frame trace %s", enabled != 0U ? "on" : "off");
        return PX4_OK;
    }

    if (strcmp(argv[0], "move") == 0) {
        if (argc != 4 || argv[1] == nullptr || argv[2] == nullptr ||
            argv[3] == nullptr) {
            return print_usage(
                "move requires azimuth_dps pitch_dps duration_ms");
        }

        long azimuth_rate = 0L;
        long pitch_rate = 0L;
        long duration_ms = 0L;
        if (!parse_long(argv[1], kJoystickRateMinimum,
                        kJoystickRateMaximum, azimuth_rate) ||
            !parse_long(argv[2], kJoystickRateMinimum,
                        kJoystickRateMaximum, pitch_rate) ||
            !parse_long(argv[3], kJoystickDurationMinimumMs,
                        kJoystickDurationMaximumMs, duration_ms)) {
            PX4_ERR("move ranges: azimuth/pitch -100..100 deg/s, duration 40..60000 ms");
            return PX4_ERROR;
        }

        if (!instance->queue_joystick_request(
                kJoystickRequestMove,
                static_cast<int16_t>(azimuth_rate),
                static_cast<int16_t>(pitch_rate),
                static_cast<uint32_t>(duration_ms))) {
            PX4_WARN("manual-move mailbox busy");
            return PX4_ERROR;
        }

        PX4_INFO("manual move queued: azimuth=%ld deg/s pitch=%ld deg/s duration=%ld ms",
                 azimuth_rate, pitch_rate, duration_ms);
        return PX4_OK;
    }

    if (strcmp(argv[0], "move_stop") == 0) {
        if (argc != 1) {
            return print_usage("move_stop takes no arguments");
        }
        if (!instance->queue_joystick_request(
                kJoystickRequestStop, 0, 0, 0U)) {
            PX4_WARN("manual-move mailbox busy");
            return PX4_ERROR;
        }
        PX4_INFO("manual move stop queued");
        return PX4_OK;
    }

    cgtd070::Command command = cgtd070::Command::Idle;
    if (strcmp(argv[0], "track") == 0) {
        command = cgtd070::Command::GreenBoxTracking;
    } else if (strcmp(argv[0], "center") == 0) {
        command = cgtd070::Command::Center;
    } else if (strcmp(argv[0], "yaw_follow") == 0) {
        command = cgtd070::Command::YawFollowAircraft;
    } else if (strcmp(argv[0], "cancel") == 0) {
        command = cgtd070::Command::CancelTracking;
    } else {
        return print_usage("unknown command");
    }

    if (!instance->queue_command(command)) {
        PX4_WARN("command mailbox busy");
        return PX4_ERROR;
    }
    return PX4_OK;
}

bool Cgtd070App::parse_config(int argc, char *argv[], Config &config) noexcept
{
    memset(&config, 0, sizeof(config));
    copy_text(config.serial_device, sizeof(config.serial_device), "/dev/ttyS3");

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            copy_text(config.serial_device, sizeof(config.serial_device), argv[++i]);
        } else {
            PX4_ERR("unknown or incomplete option: %s", argv[i]);
            return false;
        }
    }
    return true;
}

int Cgtd070App::print_usage(const char *reason)
{
    if (reason != nullptr) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
CGTD070 UART protocol application running in an independent PX4 Task.
It receives and publishes decoded feedback and emits one complete control
frame every 40 ms. The protocol core is transport-independent.
RC control uses mapped manual-control AUX2: low/middle/high is
cancel/safe/track. Entering AUX3 high queues one center command. AUX1 is not
consumed. AUX4 endpoint positions command fixed-rate gimbal azimuth movement;
AUX5 endpoint positions command fixed-rate pitch movement. Both knobs must
first return to their middle region after startup or another gimbal command.
The center NSH command also remains available.
)DESCR_STR");
    PRINT_MODULE_USAGE_NAME("cgtd070", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "device", "Serial device", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("track", "queue one 93H green-box tracking frame");
    PRINT_MODULE_USAGE_COMMAND_DESCR("center", "queue one 71H center frame");
    PRINT_MODULE_USAGE_COMMAND_DESCR("yaw_follow", "queue one 7BH yaw-follow-aircraft frame");
    PRINT_MODULE_USAGE_COMMAND_DESCR("cancel", "queue one 3BH cancel-tracking frame");
    PRINT_MODULE_USAGE_COMMAND_DESCR("move <az_dps> <pitch_dps> <duration_ms>",
                                     "timed manual movement, rates -100..100, duration 40..60000");
    PRINT_MODULE_USAGE_COMMAND_DESCR("move_stop", "stop manual movement immediately");
    PRINT_MODULE_USAGE_COMMAND_DESCR("trace on|off", "print completed non-idle 44-byte TX frames");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return PX4_OK;
}

extern "C" __EXPORT int cgtd070_main(int argc, char *argv[])
{
    return Cgtd070App::main(argc, argv);
}
