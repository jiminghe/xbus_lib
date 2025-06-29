#include "serial_reader.h"
#include "xbus/xbus.h"
#include "xbus/xbus_parser.h"
#include "xbus/xbus_message_id.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cstring>
#include <iomanip> 

class XbusMessageProcessor {
private:
    SerialReader m_serial;
    std::vector<uint8_t> m_buffer;
    bool m_running;
    
    // Message synchronization
    enum class SyncState {
        WaitingForPreamble,
        ReadingMessage
    };
    
    SyncState m_syncState;
    std::vector<uint8_t> m_messageBuffer;
    size_t m_expectedLength;
    
public:
    XbusMessageProcessor() : m_running(false), m_syncState(SyncState::WaitingForPreamble), m_expectedLength(0) {
        m_buffer.reserve(1024);
        m_messageBuffer.reserve(256);
    }
    
    bool initialize(const std::string& portName, DWORD baudRate = 115200) {
        if (!m_serial.open(portName, baudRate)) {
            std::cerr << "Failed to open serial port: " << m_serial.getLastError() << std::endl;
            return false;
        }
        
        std::cout << "Serial port " << portName << " opened successfully at " << baudRate << " baud." << std::endl;
        
        // Set up async data callback
        m_serial.setDataCallback([this](const uint8_t* data, size_t length) {
            processIncomingData(data, length);
        });
        
        return true;
    }
    
    void start() {
        if (!m_serial.isOpen()) {
            std::cerr << "Serial port is not open!" << std::endl;
            return;
        }
        
        m_running = true;
        
        if (!m_serial.startAsyncReading()) {
            std::cerr << "Failed to start async reading: " << m_serial.getLastError() << std::endl;
            return;
        }
        
        std::cout << "Started listening for Xbus messages..." << std::endl;
        std::cout << "Press 'q' and Enter to quit, 'i' for device info, 'c' to go to config mode, 'm' to go to measurement mode." << std::endl;
        
        // Main loop
        std::string input;
        while (m_running) {
            if (std::cin >> input) {
                if (input == "q" || input == "Q") {
                    m_running = false;
                } else if (input == "i" || input == "I") {
                    requestDeviceInfo();
                } else if (input == "c" || input == "C") {
                    gotoConfigMode();
                } else if (input == "m" || input == "M") {
                    gotoMeasurementMode();
                } else if (input == "f" || input == "F") {
                    requestFirmwareRevision();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void stop() {
        m_running = false;
        m_serial.stopAsyncReading();
        m_serial.close();
        std::cout << "Stopped and closed serial port." << std::endl;
    }
    
private:
    void processIncomingData(const uint8_t* data, size_t length) {
        for (size_t i = 0; i < length; i++) {
            uint8_t byte = data[i];
            
            switch (m_syncState) {
                case SyncState::WaitingForPreamble:
                    if (byte == Xbus::XBUS_PREAMBLE) {
                        m_messageBuffer.clear();
                        m_messageBuffer.push_back(byte);
                        m_syncState = SyncState::ReadingMessage;
                        m_expectedLength = 0;
                    }
                    break;
                    
                case SyncState::ReadingMessage:
                    m_messageBuffer.push_back(byte);
                    
                    // Check if we have enough bytes to determine message length
                    if (m_messageBuffer.size() >= 4 && m_expectedLength == 0) {
                        m_expectedLength = Xbus::getRawLength(m_messageBuffer.data());
                        
                        // Validate the calculated length is reasonable
                        if (m_expectedLength < 5 || m_expectedLength > 1000) {
                            std::cerr << "Invalid message length: " << m_expectedLength 
                                     << ", restarting sync..." << std::endl;
                            m_syncState = SyncState::WaitingForPreamble;
                            m_expectedLength = 0;
                            continue;
                        }
                    }
                    
                    // Check if we have a complete message
                    if (m_expectedLength > 0 && m_messageBuffer.size() >= m_expectedLength) {
                        processCompleteMessage();
                        m_syncState = SyncState::WaitingForPreamble;
                        m_expectedLength = 0;
                    }
                    
                    // Safety check - if message buffer gets too large, reset
                    if (m_messageBuffer.size() > 1000) {
                        std::cerr << "Message buffer overflow, restarting sync..." << std::endl;
                        m_syncState = SyncState::WaitingForPreamble;
                        m_expectedLength = 0;
                    }
                    break;
            }
        }
    }
    
    void processCompleteMessage() {
        if (m_messageBuffer.empty()) return;
        
        // Verify checksum
        if (!Xbus::verifyChecksum(m_messageBuffer.data())) {
            std::cerr << "Checksum verification failed!" << std::endl;
            return;
        }
        
        // Parse and display the message
        std::string messageStr = XbusParser::messageToString(m_messageBuffer.data());
        std::cout << "Received: " << messageStr << std::endl;
        
        // Special handling for XMID_MtData2 with detailed data
        uint8_t messageId = Xbus::getMessageId(m_messageBuffer.data());
        if (messageId == XMID_MtData2) {
            SensorData sensorData;
            if (XbusParser::parseMTData2(m_messageBuffer.data(), sensorData)) {
                // Display detailed breakdown
                std::cout << "  -> Detailed Data:" << std::endl;
                
                if (sensorData.hasPacketCounter) {
                    std::cout << "     Packet Counter: " << sensorData.packetCounter << std::endl;
                }
                
                if (sensorData.hasSampleTimeFine) {
                    std::cout << "     Sample Time Fine: " << sensorData.sampleTimeFine 
                             << " (approx " << (sensorData.sampleTimeFine / 10000.0) << " ms)" << std::endl;
                }
                
                if (sensorData.hasEulerAngles) {
                    std::cout << "     Euler Angles: Roll=" << std::fixed << std::setprecision(3) 
                             << sensorData.eulerAngles.roll << " deg, Pitch=" 
                             << sensorData.eulerAngles.pitch << " deg, Yaw=" 
                             << sensorData.eulerAngles.yaw << " deg" << std::endl;
                }
                
                if (sensorData.hasLatLon) {
                    std::cout << "     Position: Lat=" << std::fixed << std::setprecision(8) 
                             << sensorData.latLon.latitude << " deg, Lon=" 
                             << sensorData.latLon.longitude << " deg" << std::endl;
                }
                
                if (sensorData.hasAltitudeEllipsoid) {
                    std::cout << "     Altitude: " << std::fixed << std::setprecision(3) 
                             << sensorData.altitudeEllipsoid << " m" << std::endl;
                }
                
                if (sensorData.hasVelocityXYZ) {
                    std::cout << "     Velocity: X=" << std::fixed << std::setprecision(4) 
                             << sensorData.velocityXYZ.velX << " m/s, Y=" 
                             << sensorData.velocityXYZ.velY << " m/s, Z=" 
                             << sensorData.velocityXYZ.velZ << " m/s" << std::endl;
                }
                
                if (sensorData.hasStatusWord) {
                    std::cout << "     Status Word: 0x" << std::hex << std::uppercase 
                             << std::setfill('0') << std::setw(8) << sensorData.statusWord 
                             << std::dec << std::endl;
                }
            }
        }
    }
    
    void sendMessage(uint8_t messageId, const uint8_t* payload = nullptr, uint16_t payloadLength = 0) {
        std::vector<uint8_t> message(32); // Allocate enough space
        
        // Create the message
        Xbus::createMessage(message.data(), Xbus::XBUS_MASTERDEVICE, messageId, payloadLength);
        
        // Copy payload if provided
        if (payload && payloadLength > 0) {
            uint8_t* payloadPtr = Xbus::getPointerToPayload(message.data());
            memcpy(payloadPtr, payload, payloadLength);
        }
        
        // Insert checksum
        Xbus::insertChecksum(message.data());
        
        // Get actual message length and resize vector
        int actualLength = Xbus::getRawLength(message.data());
        message.resize(actualLength);
        
        // Create raw message for transmission
        std::vector<uint8_t> rawMessage(actualLength + 10);
        size_t rawLength = Xbus::createRawMessage(rawMessage.data(), message.data());
        rawMessage.resize(rawLength);
        
        // Send the message
        if (m_serial.write(rawMessage)) {
            std::cout << "Sent message ID: 0x" << std::hex << static_cast<int>(messageId) << std::dec << std::endl;
        } else {
            std::cerr << "Failed to send message: " << m_serial.getLastError() << std::endl;
        }
    }
    
    void requestDeviceInfo() {
        std::cout << "Requesting device ID..." << std::endl;
        sendMessage(XMID_ReqDid);
    }
    
    void gotoConfigMode() {
        std::cout << "Going to config mode..." << std::endl;
        sendMessage(XMID_GotoConfig);
    }
    
    void gotoMeasurementMode() {
        std::cout << "Going to measurement mode..." << std::endl;
        sendMessage(XMID_GotoMeasurement);
    }
    
    void requestFirmwareRevision() {
        std::cout << "Requesting firmware revision..." << std::endl;
        sendMessage(XMID_ReqFirmwareRevision);
    }
};

int main() {
    std::cout << "Xbus Serial Reader" << std::endl;
    std::cout << "==================" << std::endl;
    
    XbusMessageProcessor processor;
    
    // Initialize with COM9 at 115200 baud (8N1 is default)
    if (!processor.initialize("COM9")) {
        std::cerr << "Failed to initialize. Make sure COM9 is available and not in use." << std::endl;
        std::cout << "Press Enter to exit...";
        std::cin.get();
        return 1;
    }
    
    // Start processing
    processor.start();
    
    // Cleanup
    processor.stop();
    
    return 0;
}