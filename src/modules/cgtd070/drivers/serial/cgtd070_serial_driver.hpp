#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cgtd070 {

enum class SerialOpenStage : uint8_t {
    None = 0U,
    OpenDevice = 1U,
    GetAttributes = 2U,
    SetAttributes = 3U,
};

class SerialDriver final {
public:
    SerialDriver() noexcept;
    ~SerialDriver();

    SerialDriver(const SerialDriver &) = delete;
    SerialDriver &operator=(const SerialDriver &) = delete;

    // Must be called by the same PX4 task that later calls read/write/close.
    bool open(const char *device) noexcept;
    void close() noexcept;

    bool is_open() const noexcept;
    int fd() const noexcept;

    // Return value: >0 bytes transferred, 0 means no progress yet, -1 means
    // a hard I/O error. EAGAIN/EINTR are reported as no progress. On NuttX,
    // EWOULDBLOCK is the same value as EAGAIN.
    int read_available(uint8_t *buffer, size_t capacity) noexcept;
    int write_some(const uint8_t *data, size_t length) noexcept;

    SerialOpenStage open_stage() const noexcept;
    int open_error() const noexcept;
    int read_error() const noexcept;
    int write_error() const noexcept;
    uint32_t read_errors() const noexcept;
    uint32_t write_errors() const noexcept;
    uint32_t write_would_block() const noexcept;

private:
    int fd_;
    uint8_t open_stage_;
    int open_error_;
    int read_error_;
    int write_error_;
    uint32_t read_errors_;
    uint32_t write_errors_;
    uint32_t write_would_block_;
};

} // namespace cgtd070
