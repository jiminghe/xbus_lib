#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int  fd;
    bool is_open;
    char last_error[256];
} SerialReader;

void serial_reader_init(SerialReader* r);

/* Open `port_name` (e.g. "/dev/ttyUSB0"). parity: 'N'/'E'/'O'. */
bool serial_reader_open(SerialReader* r, const char* port_name, int baud_rate,
                        int data_bits, char parity, int stop_bits);

void serial_reader_close(SerialReader* r);
bool serial_reader_is_open(const SerialReader* r);

bool serial_reader_write(SerialReader* r, const uint8_t* data, size_t length);

/* Non-blocking read of whatever is currently buffered. Returns bytes read,
   0 if nothing pending, -1 on error. */
int  serial_reader_read_available(SerialReader* r, uint8_t* buffer, size_t buffer_size);

bool        serial_reader_flush     (SerialReader* r);
const char* serial_reader_last_error(const SerialReader* r);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_READER_H */
