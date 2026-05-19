#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <cstdint>
#include <cstddef>

class SerialReader {
public:
    SerialReader();
    ~SerialReader();

    // Open serial port with specified parameters.
    // parity: 'N' (none), 'E' (even), 'O' (odd).
    bool open(const std::string& portName, int baudRate = 115200,
              int dataBits = 8, char parity = 'N', int stopBits = 1);

    void close();
    bool isOpen() const;

    bool write(const uint8_t* data, size_t length);
    bool write(const std::vector<uint8_t>& data);

    // Blocking read up to timeoutMs milliseconds.
    int read(uint8_t* buffer, size_t bufferSize, int timeoutMs = 1000);

    // Non-blocking read of whatever is currently buffered.
    int readAvailable(uint8_t* buffer, size_t bufferSize);

    void setDataCallback(std::function<void(const uint8_t*, size_t)> callback);

    bool startAsyncReading();
    void stopAsyncReading();

    std::string getLastError() const;
    bool flushBuffers();

private:
    int m_fd;
    bool m_isOpen;
    std::string m_lastError;

    std::thread m_readThread;
    std::atomic<bool> m_stopReading;
    std::function<void(const uint8_t*, size_t)> m_dataCallback;

    void readLoop();
    void setLastError(const std::string& error);
    bool setupSerialPort(int baudRate, int dataBits, char parity, int stopBits);
};

#endif // SERIAL_READER_H
