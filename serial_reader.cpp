#include "serial_reader.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <errno.h>
#include <string.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace {

speed_t baudRateToSpeed(int baudRate) {
    switch (baudRate) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        default:      return 0; // unsupported
    }
}

} // namespace

SerialReader::SerialReader()
    : m_fd(-1)
    , m_isOpen(false)
    , m_stopReading(false) {
}

SerialReader::~SerialReader() {
    close();
}

bool SerialReader::open(const std::string& portName, int baudRate,
                        int dataBits, char parity, int stopBits) {
    if (m_isOpen) {
        setLastError("Port is already open");
        return false;
    }

    m_fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        setLastError("Failed to open port " + portName + ": " + std::string(strerror(errno)));
        return false;
    }

    if (!setupSerialPort(baudRate, dataBits, parity, stopBits)) {
        ::close(m_fd);
        m_fd = -1;
        return false;
    }

    m_isOpen = true;
    flushBuffers();
    return true;
}

void SerialReader::close() {
    if (m_isOpen) {
        stopAsyncReading();

        if (m_fd >= 0) {
            ::close(m_fd);
            m_fd = -1;
        }
        m_isOpen = false;
    }
}

bool SerialReader::isOpen() const {
    return m_isOpen;
}

bool SerialReader::write(const uint8_t* data, size_t length) {
    if (!m_isOpen) {
        setLastError("Port is not open");
        return false;
    }

    size_t total = 0;
    while (total < length) {
        ssize_t n = ::write(m_fd, data + total, length - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Wait briefly for output buffer space.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            setLastError(std::string("Failed to write data: ") + strerror(errno));
            return false;
        }
        total += static_cast<size_t>(n);
    }
    return true;
}

bool SerialReader::write(const std::vector<uint8_t>& data) {
    return write(data.data(), data.size());
}

int SerialReader::read(uint8_t* buffer, size_t bufferSize, int timeoutMs) {
    if (!m_isOpen) {
        setLastError("Port is not open");
        return -1;
    }

    struct pollfd pfd;
    pfd.fd = m_fd;
    pfd.events = POLLIN;

    int rc = ::poll(&pfd, 1, timeoutMs);
    if (rc < 0) {
        if (errno == EINTR) return 0;
        setLastError(std::string("poll failed: ") + strerror(errno));
        return -1;
    }
    if (rc == 0) {
        return 0; // timeout
    }

    ssize_t n = ::read(m_fd, buffer, bufferSize);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        setLastError(std::string("Failed to read data: ") + strerror(errno));
        return -1;
    }
    return static_cast<int>(n);
}

int SerialReader::readAvailable(uint8_t* buffer, size_t bufferSize) {
    if (!m_isOpen) {
        setLastError("Port is not open");
        return -1;
    }

    int available = 0;
    if (ioctl(m_fd, FIONREAD, &available) < 0) {
        setLastError(std::string("ioctl(FIONREAD) failed: ") + strerror(errno));
        return -1;
    }
    if (available <= 0) {
        return 0;
    }

    size_t toRead = std::min(bufferSize, static_cast<size_t>(available));
    ssize_t n = ::read(m_fd, buffer, toRead);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        setLastError(std::string("Failed to read data: ") + strerror(errno));
        return -1;
    }
    return static_cast<int>(n);
}

void SerialReader::setDataCallback(std::function<void(const uint8_t*, size_t)> callback) {
    m_dataCallback = std::move(callback);
}

bool SerialReader::startAsyncReading() {
    if (!m_isOpen) {
        setLastError("Port is not open");
        return false;
    }
    if (m_readThread.joinable()) {
        setLastError("Async reading is already started");
        return false;
    }

    m_stopReading = false;
    m_readThread = std::thread([this]() { readLoop(); });
    return true;
}

void SerialReader::stopAsyncReading() {
    if (m_readThread.joinable()) {
        m_stopReading = true;
        m_readThread.join();
    }
}

std::string SerialReader::getLastError() const {
    return m_lastError;
}

bool SerialReader::flushBuffers() {
    if (!m_isOpen) {
        return false;
    }
    return tcflush(m_fd, TCIOFLUSH) == 0;
}

void SerialReader::readLoop() {
    uint8_t buffer[1024];

    while (!m_stopReading && m_isOpen) {
        struct pollfd pfd;
        pfd.fd = m_fd;
        pfd.events = POLLIN;

        int rc = ::poll(&pfd, 1, 50); // 50 ms wake-up to check m_stopReading
        if (rc < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (rc == 0) continue;

        ssize_t n = ::read(m_fd, buffer, sizeof(buffer));
        if (n > 0 && m_dataCallback) {
            m_dataCallback(buffer, static_cast<size_t>(n));
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            break;
        }
    }
}

void SerialReader::setLastError(const std::string& error) {
    m_lastError = error;
}

bool SerialReader::setupSerialPort(int baudRate, int dataBits, char parity, int stopBits) {
    struct termios tio;
    if (tcgetattr(m_fd, &tio) != 0) {
        setLastError(std::string("tcgetattr failed: ") + strerror(errno));
        return false;
    }

    speed_t speed = baudRateToSpeed(baudRate);
    if (speed == 0) {
        setLastError("Unsupported baud rate: " + std::to_string(baudRate));
        return false;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    // Raw 8N1 by default; adjusted by parameters below.
    cfmakeraw(&tio);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    switch (dataBits) {
        case 5: tio.c_cflag |= CS5; break;
        case 6: tio.c_cflag |= CS6; break;
        case 7: tio.c_cflag |= CS7; break;
        case 8: tio.c_cflag |= CS8; break;
        default:
            setLastError("Unsupported data bits: " + std::to_string(dataBits));
            return false;
    }

    switch (parity) {
        case 'N': case 'n':
            tio.c_cflag &= ~PARENB;
            tio.c_iflag &= ~INPCK;
            break;
        case 'E': case 'e':
            tio.c_cflag |= PARENB;
            tio.c_cflag &= ~PARODD;
            tio.c_iflag |= INPCK;
            break;
        case 'O': case 'o':
            tio.c_cflag |= (PARENB | PARODD);
            tio.c_iflag |= INPCK;
            break;
        default:
            setLastError(std::string("Unsupported parity: ") + parity);
            return false;
    }

    if (stopBits == 2) {
        tio.c_cflag |= CSTOPB;
    } else {
        tio.c_cflag &= ~CSTOPB;
    }

    // Disable hardware flow control.
    tio.c_cflag &= ~CRTSCTS;
    // Disable software flow control.
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Non-blocking VMIN/VTIME so read() returns immediately when no data;
    // poll() drives the actual waiting.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
        setLastError(std::string("tcsetattr failed: ") + strerror(errno));
        return false;
    }

    return true;
}
