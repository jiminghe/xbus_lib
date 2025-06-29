#include "serial_reader.h"
#include <iostream>
#include <algorithm>

SerialReader::SerialReader() 
    : m_hSerial(INVALID_HANDLE_VALUE)
    , m_isOpen(false)
    , m_hReadThread(nullptr)
    , m_stopReading(false) {
}

SerialReader::~SerialReader() {
    close();
}

bool SerialReader::open(const std::string& portName, DWORD baudRate, 
                       BYTE dataBits, BYTE parity, BYTE stopBits) {
    if (m_isOpen) {
        setLastError("Port is already open");
        return false;
    }
    
    // Open the serial port
    std::string fullPortName = "\\\\.\\" + portName;
    m_hSerial = CreateFileA(fullPortName.c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0,
                           nullptr,
                           OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL,
                           nullptr);
    
    if (m_hSerial == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        setLastError("Failed to open port " + portName + ". Error code: " + std::to_string(error));
        return false;
    }
    
    // Setup the serial port parameters
    if (!setupSerialPort(baudRate, dataBits, parity, stopBits)) {
        CloseHandle(m_hSerial);
        m_hSerial = INVALID_HANDLE_VALUE;
        return false;
    }
    
    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(m_hSerial, &timeouts)) {
        setLastError("Failed to set timeouts");
        CloseHandle(m_hSerial);
        m_hSerial = INVALID_HANDLE_VALUE;
        return false;
    }
    
    // Flush any existing data
    flushBuffers();
    
    m_isOpen = true;
    return true;
}

void SerialReader::close() {
    if (m_isOpen) {
        stopAsyncReading();
        
        if (m_hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(m_hSerial);
            m_hSerial = INVALID_HANDLE_VALUE;
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
    
    DWORD bytesWritten = 0;
    if (!WriteFile(m_hSerial, data, static_cast<DWORD>(length), &bytesWritten, nullptr)) {
        setLastError("Failed to write data");
        return false;
    }
    
    return bytesWritten == length;
}

bool SerialReader::write(const std::vector<uint8_t>& data) {
    return write(data.data(), data.size());
}

int SerialReader::read(uint8_t* buffer, size_t bufferSize, DWORD timeoutMs) {
    if (!m_isOpen) {
        setLastError("Port is not open");
        return -1;
    }
    
    // Set timeout for this read operation
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = timeoutMs;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    
    SetCommTimeouts(m_hSerial, &timeouts);
    
    DWORD bytesRead = 0;
    if (!ReadFile(m_hSerial, buffer, static_cast<DWORD>(bufferSize), &bytesRead, nullptr)) {
        setLastError("Failed to read data");
        return -1;
    }
    
    return static_cast<int>(bytesRead);
}

int SerialReader::readAvailable(uint8_t* buffer, size_t bufferSize) {
    if (!m_isOpen) {
        setLastError("Port is not open");
        return -1;
    }
    
    DWORD bytesRead = 0;
    DWORD errors = 0;
    COMSTAT comStat;
    
    // Check how many bytes are available
    if (!ClearCommError(m_hSerial, &errors, &comStat)) {
        setLastError("Failed to get comm status");
        return -1;
    }
    
    if (comStat.cbInQue == 0) {
        return 0; // No data available
    }
    
    DWORD toRead = std::min(static_cast<DWORD>(bufferSize), comStat.cbInQue);
    
    if (!ReadFile(m_hSerial, buffer, toRead, &bytesRead, nullptr)) {
        setLastError("Failed to read data");
        return -1;
    }
    
    return static_cast<int>(bytesRead);
}

void SerialReader::setDataCallback(std::function<void(const uint8_t*, size_t)> callback) {
    m_dataCallback = callback;
}

bool SerialReader::startAsyncReading() {
    if (!m_isOpen) {
        setLastError("Port is not open");
        return false;
    }
    
    if (m_hReadThread != nullptr) {
        setLastError("Async reading is already started");
        return false;
    }
    
    m_stopReading = false;
    m_hReadThread = CreateThread(nullptr, 0, readThreadProc, this, 0, nullptr);
    
    if (m_hReadThread == nullptr) {
        setLastError("Failed to create read thread");
        return false;
    }
    
    return true;
}

void SerialReader::stopAsyncReading() {
    if (m_hReadThread != nullptr) {
        m_stopReading = true;
        WaitForSingleObject(m_hReadThread, 2000); // Wait up to 2 seconds
        CloseHandle(m_hReadThread);
        m_hReadThread = nullptr;
    }
}

std::string SerialReader::getLastError() const {
    return m_lastError;
}

bool SerialReader::flushBuffers() {
    if (!m_isOpen) {
        return false;
    }
    
    return PurgeComm(m_hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR) != 0;
}

DWORD WINAPI SerialReader::readThreadProc(LPVOID lpParam) {
    SerialReader* reader = static_cast<SerialReader*>(lpParam);
    reader->readLoop();
    return 0;
}

void SerialReader::readLoop() {
    uint8_t buffer[1024];
    
    while (!m_stopReading && m_isOpen) {
        int bytesRead = readAvailable(buffer, sizeof(buffer));
        
        if (bytesRead > 0 && m_dataCallback) {
            m_dataCallback(buffer, bytesRead);
        }
        
        Sleep(10); // Small delay to prevent excessive CPU usage
    }
}

void SerialReader::setLastError(const std::string& error) {
    m_lastError = error;
}

bool SerialReader::setupSerialPort(DWORD baudRate, BYTE dataBits, BYTE parity, BYTE stopBits) {
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    
    if (!GetCommState(m_hSerial, &dcb)) {
        setLastError("Failed to get current DCB");
        return false;
    }
    
    dcb.BaudRate = baudRate;
    dcb.ByteSize = dataBits;
    dcb.Parity = parity;
    dcb.StopBits = stopBits;
    dcb.fBinary = TRUE;
    dcb.fParity = (parity != NOPARITY);
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;
    
    if (!SetCommState(m_hSerial, &dcb)) {
        setLastError("Failed to set DCB");
        return false;
    }
    
    return true;
}