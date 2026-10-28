#pragma once

#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cgtd070_status.h>
#include <uORB/topics/cgtd070_diag.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>

#include "cgtd070_core.hpp"
#include "cgtd070_serial_driver.hpp"

class Cgtd070App final : public ModuleBase<Cgtd070App>
{
public:
    struct Config {
        char serial_device[32];
    };

    explicit Cgtd070App(const Config &config) noexcept;
    ~Cgtd070App() override;

    static int task_spawn(int argc, char *argv[]);
    static Cgtd070App *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;
    int print_status() override;

private:
    static constexpr uint64_t kTxIntervalUs = 40000U;
    static constexpr uint64_t kDiagIntervalUs = 100000U; // 10 Hz
    static constexpr int kMaxPollTimeoutMs = 20;
    static constexpr int kTaskStackSize = 4096;

    static bool parse_config(int argc, char *argv[], Config &config) noexcept;
    bool open_serial() noexcept;
    bool queue_command(cgtd070::Command command) noexcept;
    void service_rx() noexcept;
    void prepare_tx_if_due(uint64_t now) noexcept;
    void service_tx() noexcept;
    void trace_completed_command_frame() const noexcept;
    int poll_timeout_ms(uint64_t now) const noexcept;
    void update_flight_sources() noexcept;
    void update_rc_control(uint64_t now) noexcept;
    void reset_rc_state() noexcept;
    cgtd070::FlightTelemetry make_flight_telemetry(uint64_t now) const noexcept;
    void process_feedback(const cgtd070::Feedback &feedback) noexcept;
    void update_parser_snapshot() noexcept;
    void publish_diag(uint64_t now) noexcept;

    Config config_;
    cgtd070::Core core_;
    cgtd070::SerialDriver serial_;
    uORB::Publication<cgtd070_status_s> status_pub_{ORB_ID(cgtd070_status)};
    uORB::Publication<cgtd070_diag_s> diag_pub_{ORB_ID(cgtd070_diag)};
    uORB::Subscription attitude_sub_{ORB_ID(vehicle_attitude)};
    uORB::Subscription gps_sub_{ORB_ID(sensor_gps)};
    uORB::Subscription battery_sub_{ORB_ID(battery_status)};
    uORB::Subscription global_position_sub_{ORB_ID(vehicle_global_position)};
    uORB::Subscription home_position_sub_{ORB_ID(home_position)};
    uORB::Subscription manual_control_sub_{ORB_ID(manual_control_setpoint)};

    vehicle_attitude_s attitude_{};
    sensor_gps_s gps_{};
    battery_status_s battery_{};
    vehicle_global_position_s global_position_{};
    home_position_s home_position_{};
    manual_control_setpoint_s manual_control_{};

    int8_t rc_switch_stable_;
    int8_t rc_switch_candidate_;
    uint64_t rc_switch_candidate_since_;
    uint8_t rc_center_stable_;
    uint8_t rc_center_candidate_;
    uint64_t rc_center_candidate_since_;
    uint8_t rc_state_initialized_;
    uint8_t rc_input_valid_;
    int32_t rc_aux2_milli_;
    int32_t rc_aux3_milli_;
    int32_t rc_switch_status_;
    uint32_t rc_center_status_;
    uint32_t rc_commands_;
    uint32_t rc_command_busy_;

    uint8_t tx_buffer_[cgtd070::kControlFrameSize];
    size_t tx_offset_;
    uint64_t next_tx_us_;

    // Written by the module Task and read by NSH status. Atomic builtins avoid
    // data races without STL, mutexes, or additional dynamic allocation.
    uint32_t tx_frames_;
    uint32_t rx_frames_;
    uint32_t io_errors_;
    uint32_t poll_errors_;
    int poll_error_;
    uint32_t missed_tx_slots_;
    uint32_t parser_bytes_;
    uint32_t parser_ok_;
    uint32_t parser_checksum_;
    uint32_t parser_framing_;
    uint32_t parser_overruns_;
    uint32_t feedback_counter_;
    uint32_t feedback_status_primary_;
    uint32_t feedback_status_secondary_;
    uint32_t feedback_servo_state_;
    uint8_t command_mailbox_;
    uint8_t have_feedback_;
    uint8_t trace_enabled_;

    // ULog diagnostic state. These are used only by the cgtd070 Task.
    uint32_t telemetry_frames_;
    uint64_t last_rx_byte_time_;
    uint64_t last_parsed_frame_time_;
    uint64_t last_telemetry_time_;
    uint64_t last_diag_publish_time_;
};
