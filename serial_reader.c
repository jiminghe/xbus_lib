#define _DEFAULT_SOURCE          /* cfmakeraw, CRTSCTS */
#define _POSIX_C_SOURCE 200809L  /* ssize_t */

#include "serial_reader.h"

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

static void set_last_errorf(SerialReader* r, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(r->last_error, sizeof(r->last_error), fmt, ap);
    va_end(ap);
}

static speed_t baud_to_speed(int baud) {
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return 0;
    }
}

static bool setup_termios(SerialReader* r, int baud, int data_bits, char parity, int stop_bits) {
    struct termios tio;
    if (tcgetattr(r->fd, &tio) != 0) {
        set_last_errorf(r, "tcgetattr failed: %s", strerror(errno));
        return false;
    }

    speed_t speed = baud_to_speed(baud);
    if (speed == 0) {
        set_last_errorf(r, "Unsupported baud rate: %d", baud);
        return false;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    cfmakeraw(&tio);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    switch (data_bits) {
    case 5: tio.c_cflag |= CS5; break;
    case 6: tio.c_cflag |= CS6; break;
    case 7: tio.c_cflag |= CS7; break;
    case 8: tio.c_cflag |= CS8; break;
    default:
        set_last_errorf(r, "Unsupported data bits: %d", data_bits);
        return false;
    }

    switch (parity) {
    case 'N': case 'n':
        tio.c_cflag &= ~PARENB;
        tio.c_iflag &= ~INPCK;
        break;
    case 'E': case 'e':
        tio.c_cflag |=  PARENB;
        tio.c_cflag &= ~PARODD;
        tio.c_iflag |=  INPCK;
        break;
    case 'O': case 'o':
        tio.c_cflag |= (PARENB | PARODD);
        tio.c_iflag |=  INPCK;
        break;
    default:
        set_last_errorf(r, "Unsupported parity: %c", parity);
        return false;
    }
    if (stop_bits == 2) tio.c_cflag |=  CSTOPB;
    else                tio.c_cflag &= ~CSTOPB;

    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(r->fd, TCSANOW, &tio) != 0) {
        set_last_errorf(r, "tcsetattr failed: %s", strerror(errno));
        return false;
    }
    return true;
}

void serial_reader_init(SerialReader* r) {
    memset(r, 0, sizeof(*r));
    r->fd = -1;
}

bool serial_reader_open(SerialReader* r, const char* port, int baud,
                        int data_bits, char parity, int stop_bits) {
    if (r->is_open) {
        set_last_errorf(r, "Port is already open");
        return false;
    }
    r->fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (r->fd < 0) {
        set_last_errorf(r, "Failed to open port %s: %s", port, strerror(errno));
        return false;
    }
    if (!setup_termios(r, baud, data_bits, parity, stop_bits)) {
        close(r->fd);
        r->fd = -1;
        return false;
    }
    r->is_open = true;
    serial_reader_flush(r);
    return true;
}

void serial_reader_close(SerialReader* r) {
    if (!r->is_open) return;
    if (r->fd >= 0) { close(r->fd); r->fd = -1; }
    r->is_open = false;
}

bool serial_reader_is_open(const SerialReader* r) {
    return r->is_open;
}

bool serial_reader_write(SerialReader* r, const uint8_t* data, size_t length) {
    if (!r->is_open) {
        set_last_errorf(r, "Port is not open");
        return false;
    }
    size_t total = 0;
    while (total < length) {
        ssize_t n = write(r->fd, data + total, length - total);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            set_last_errorf(r, "Failed to write data: %s", strerror(errno));
            return false;
        }
        total += (size_t)n;
    }
    return true;
}

int serial_reader_read_available(SerialReader* r, uint8_t* buffer, size_t buffer_size) {
    if (!r->is_open) {
        set_last_errorf(r, "Port is not open");
        return -1;
    }
    int available = 0;
    if (ioctl(r->fd, FIONREAD, &available) < 0) {
        set_last_errorf(r, "ioctl(FIONREAD) failed: %s", strerror(errno));
        return -1;
    }
    if (available <= 0) return 0;

    size_t to_read = (size_t)available < buffer_size ? (size_t)available : buffer_size;
    ssize_t n = read(r->fd, buffer, to_read);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        set_last_errorf(r, "Failed to read data: %s", strerror(errno));
        return -1;
    }
    return (int)n;
}

bool serial_reader_flush(SerialReader* r) {
    if (!r->is_open) return false;
    return tcflush(r->fd, TCIOFLUSH) == 0;
}

const char* serial_reader_last_error(const SerialReader* r) {
    return r->last_error;
}
