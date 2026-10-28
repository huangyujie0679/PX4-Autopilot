#include "cgtd070_serial_driver.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

namespace cgtd070 {

SerialDriver::SerialDriver() noexcept :
    fd_(-1),
    open_stage_(static_cast<uint8_t>(SerialOpenStage::None)),
    open_error_(0),
    read_error_(0),
    write_error_(0),
    read_errors_(0U),
    write_errors_(0U),
    write_would_block_(0U)
{
}

SerialDriver::~SerialDriver()
{
    close();
}

bool SerialDriver::open(const char *device) noexcept
{
    close();

    __atomic_store_n(&open_stage_,
                     static_cast<uint8_t>(SerialOpenStage::None),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&open_error_, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&read_error_, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&write_error_, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&read_errors_, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&write_errors_, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&write_would_block_, 0U, __ATOMIC_RELEASE);

    if (device == nullptr || device[0] == '\0') {
        __atomic_store_n(&open_stage_,
                         static_cast<uint8_t>(SerialOpenStage::OpenDevice),
                         __ATOMIC_RELEASE);
        __atomic_store_n(&open_error_, EINVAL, __ATOMIC_RELEASE);
        return false;
    }

    // The Task owns this descriptor for its entire lifetime. Non-blocking I/O
    // keeps stop handling and the 25 Hz transmit deadline bounded.
    const int opened_fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (opened_fd < 0) {
        __atomic_store_n(&open_stage_,
                         static_cast<uint8_t>(SerialOpenStage::OpenDevice),
                         __ATOMIC_RELEASE);
        __atomic_store_n(&open_error_, errno, __ATOMIC_RELEASE);
        return false;
    }
    __atomic_store_n(&fd_, opened_fd, __ATOMIC_RELEASE);

    struct termios config;
    memset(&config, 0, sizeof(config));
    if (tcgetattr(opened_fd, &config) != 0) {
        __atomic_store_n(&open_stage_,
                         static_cast<uint8_t>(SerialOpenStage::GetAttributes),
                         __ATOMIC_RELEASE);
        __atomic_store_n(&open_error_, errno, __ATOMIC_RELEASE);
        close();
        return false;
    }

    config.c_iflag = 0;
    config.c_oflag = 0;
    config.c_lflag = 0;
    config.c_cflag = CS8 | CLOCAL | CREAD;
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 0;
    (void)cfsetispeed(&config, B115200);
    (void)cfsetospeed(&config, B115200);

    if (tcsetattr(opened_fd, TCSANOW, &config) != 0) {
        __atomic_store_n(&open_stage_,
                         static_cast<uint8_t>(SerialOpenStage::SetAttributes),
                         __ATOMIC_RELEASE);
        __atomic_store_n(&open_error_, errno, __ATOMIC_RELEASE);
        close();
        return false;
    }

    (void)tcflush(opened_fd, TCIOFLUSH);
    __atomic_store_n(&open_stage_,
                     static_cast<uint8_t>(SerialOpenStage::None),
                     __ATOMIC_RELEASE);
    __atomic_store_n(&open_error_, 0, __ATOMIC_RELEASE);
    return true;
}

void SerialDriver::close() noexcept
{
    const int descriptor = __atomic_exchange_n(&fd_, -1, __ATOMIC_ACQ_REL);
    if (descriptor >= 0) {
        (void)::close(descriptor);
    }
}

bool SerialDriver::is_open() const noexcept
{
    return __atomic_load_n(&fd_, __ATOMIC_ACQUIRE) >= 0;
}

int SerialDriver::fd() const noexcept
{
    return __atomic_load_n(&fd_, __ATOMIC_ACQUIRE);
}

int SerialDriver::read_available(uint8_t *buffer, size_t capacity) noexcept
{
    const int descriptor = fd();
    if (descriptor < 0 || buffer == nullptr || capacity == 0U) {
        __atomic_store_n(&read_error_, EINVAL, __ATOMIC_RELEASE);
        __atomic_add_fetch(&read_errors_, 1U, __ATOMIC_RELAXED);
        return -1;
    }

    const ssize_t result = ::read(descriptor, buffer, capacity);
    if (result > 0) {
        return static_cast<int>(result);
    }
    if (result == 0) {
        return 0;
    }

    const int error = errno;
    // NuttX defines EWOULDBLOCK as the same value as EAGAIN. Checking both
    // triggers PX4's -Wlogical-op warning, which is promoted to an error.
    if (error == EAGAIN || error == EINTR) {
        return 0;
    }

    __atomic_store_n(&read_error_, error, __ATOMIC_RELEASE);
    __atomic_add_fetch(&read_errors_, 1U, __ATOMIC_RELAXED);
    return -1;
}

int SerialDriver::write_some(const uint8_t *data, size_t length) noexcept
{
    const int descriptor = fd();
    if (descriptor < 0 || data == nullptr || length == 0U) {
        __atomic_store_n(&write_error_, EINVAL, __ATOMIC_RELEASE);
        __atomic_add_fetch(&write_errors_, 1U, __ATOMIC_RELAXED);
        return -1;
    }

    const ssize_t result = ::write(descriptor, data, length);
    if (result > 0) {
        return static_cast<int>(result);
    }
    if (result == 0) {
        __atomic_store_n(&write_error_, EIO, __ATOMIC_RELEASE);
        __atomic_add_fetch(&write_errors_, 1U, __ATOMIC_RELAXED);
        return -1;
    }

    const int error = errno;
    if (error == EAGAIN || error == EINTR) {
        __atomic_add_fetch(&write_would_block_, 1U, __ATOMIC_RELAXED);
        return 0;
    }

    __atomic_store_n(&write_error_, error, __ATOMIC_RELEASE);
    __atomic_add_fetch(&write_errors_, 1U, __ATOMIC_RELAXED);
    return -1;
}

SerialOpenStage SerialDriver::open_stage() const noexcept
{
    return static_cast<SerialOpenStage>(
        __atomic_load_n(&open_stage_, __ATOMIC_ACQUIRE));
}

int SerialDriver::open_error() const noexcept
{
    return __atomic_load_n(&open_error_, __ATOMIC_ACQUIRE);
}

int SerialDriver::read_error() const noexcept
{
    return __atomic_load_n(&read_error_, __ATOMIC_ACQUIRE);
}

int SerialDriver::write_error() const noexcept
{
    return __atomic_load_n(&write_error_, __ATOMIC_ACQUIRE);
}

uint32_t SerialDriver::read_errors() const noexcept
{
    return __atomic_load_n(&read_errors_, __ATOMIC_ACQUIRE);
}

uint32_t SerialDriver::write_errors() const noexcept
{
    return __atomic_load_n(&write_errors_, __ATOMIC_ACQUIRE);
}

uint32_t SerialDriver::write_would_block() const noexcept
{
    return __atomic_load_n(&write_would_block_, __ATOMIC_ACQUIRE);
}

} // namespace cgtd070
