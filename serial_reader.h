#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include <windows.h>
#include <string>
#include <vector>
#include <functional>

class SerialReader {
public:
    // Constructor
    SerialReader();
    
    // Destructor
    ~SerialReader();
    
    // Open serial port with specified parameters
    bool open(const std::string& portName, DWORD baudRate = 115200, 
              BYTE dataBits = 8, BYTE parity = NOPARITY, BYTE stopBits = ONESTOPBIT);
    
    // Close the serial port
    void close();
    
    // Check if port is open
    bool isOpen() const;
    
    // Write data to serial port
    bool write(const uint8_t* data, size_t length);
    bool write(const std::vector<uint8_t>& data);
    
    // Read data from serial port (blocking)
    int read(uint8_t* buffer, size_t bufferSize, DWORD timeoutMs = 1000);
    
    // Read data from serial port (non-blocking)
    int readAvailable(uint8_t* buffer, size_t bufferSize);
    
    // Set callback for received data (for async reading)
    void setDataCallback(std::function<void(const uint8_t*, size_t)> callback);
    
    // Start/stop async reading thread
    bool startAsyncReading();
    void stopAsyncReading();
    
    // Get last error message
    std::string getLastError() const;
    
    // Flush input/output buffers
    bool flushBuffers();
    
private:
    HANDLE m_hSerial;
    bool m_isOpen;
    std::string m_lastError;
    
    // Async reading
    HANDLE m_hReadThread;
    bool m_stopReading;
    std::function<void(const uint8_t*, size_t)> m_dataCallback;
    
    // Thread function for async reading
    static DWORD WINAPI readThreadProc(LPVOID lpParam);
    void readLoop();
    
    // Helper methods
    void setLastError(const std::string& error);
    bool setupSerialPort(DWORD baudRate, BYTE dataBits, BYTE parity, BYTE stopBits);
};

#endif // SERIAL_READER_H