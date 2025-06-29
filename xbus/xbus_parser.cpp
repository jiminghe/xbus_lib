// Enhanced xbus_parser.cpp
#include "xbus_parser.h"
#include <sstream>
#include <iomanip>
#include <cstring>

uint8_t XbusParser::readUint8(const uint8_t* data, int& index) {
    return data[index++];
}

uint16_t XbusParser::readUint16(const uint8_t* data, int& index) {
    uint16_t result = 0;
    result |= data[index++] << 8;
    result |= data[index++] << 0;
    return result;
}

uint32_t XbusParser::readUint32(const uint8_t* data, int& index) {
    uint32_t result = 0;
    result |= data[index++] << 24;
    result |= data[index++] << 16;
    result |= data[index++] << 8;
    result |= data[index++] << 0;
    return result;
}

float XbusParser::readFloat(const uint8_t* data, int& index) {
    uint32_t temp = readUint32(data, index);
    float result;
    memcpy(&result, &temp, 4);
    return result;
}

double XbusParser::readFP1632(const uint8_t* data, int& index) {
    // FP16.32 format: 32-bit fractional part followed by 16-bit integer part (6 bytes total)
    // Formula: i = round(realNumber * 2^32)
    // Range: [-32768.0 .. 32767.9999999998]
    // Transmission order: {fractional[31:24], fractional[23:16], fractional[15:8], fractional[7:0], integer[15:8], integer[7:0]}
    
    // Read 32-bit fractional part first (big-endian)
    uint32_t fractionalPart = readUint32(data, index);
    
    // Read 16-bit integer part (big-endian)
    uint16_t integerPartUnsigned = readUint16(data, index);
    // Convert to signed 16-bit value
    int16_t integerPart = static_cast<int16_t>(integerPartUnsigned);
    
    // Combine into 48-bit signed value and convert to double
    // The 48-bit value is: (integerPart << 32) | fractionalPart
    int64_t fixedPointValue = (static_cast<int64_t>(integerPart) << 32) | (static_cast<int64_t>(fractionalPart) & 0xffffffff);
    // Convert from fixed point to double: divide by 2^32
    double result = static_cast<double>(fixedPointValue) / 4294967296.0; // 2^32
    
    return result;
}

std::string XbusParser::messageToString(const uint8_t* xbusData) {
    if (!Xbus::checkPreamble(xbusData)) {
        return "Invalid xbus message";
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    int index = 4;
    std::ostringstream oss;

    switch (messageId) {
        case XMID_Wakeup:
            return "XMID_Wakeup";

        case XMID_DeviceId: {
            uint32_t deviceId = readUint32(xbusData, index);
            oss << "XMID_DeviceId: 0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << deviceId;
            return oss.str();
        }

        case XMID_GotoConfigAck:
            return "XMID_GotoConfigAck";

        case XMID_GotoMeasurementAck:
            return "XMID_GotoMeasurementAck";

        case XMID_MtData2: {
            // Use enhanced parsing for MTData2
            SensorData sensorData;
            if (parseMTData2(xbusData, sensorData)) {
                return "XMID_MtData2: " + formatSensorData(sensorData);
            } else {
                return "XMID_MtData2: Failed to parse";
            }
        }

        case XMID_FirmwareRevision: {
            uint8_t major = readUint8(xbusData, index);
            uint8_t minor = readUint8(xbusData, index);
            uint8_t patch = readUint8(xbusData, index);
            oss << "Firmware revision: " << static_cast<int>(major) << "." 
                << static_cast<int>(minor) << "." << static_cast<int>(patch);
            return oss.str();
        }

        case XMID_GotoBootLoaderAck:
            return "XMID_GotoBootLoaderAck";

        case XMID_FirmwareUpdate:
            return "XMID_FirmwareUpdate";

        case XMID_ResetAck:
            return "XMID_ResetAck";

        default:
            oss << "Unhandled xbus message: MessageId = 0x" << std::hex << std::uppercase 
                << std::setfill('0') << std::setw(2) << static_cast<int>(messageId);
            return oss.str();
    }
}

bool XbusParser::parseMTData2(const uint8_t* xbusData, SensorData& sensorData) {
    if (!Xbus::checkPreamble(xbusData)) {
        return false;
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    if (messageId != XMID_MtData2) {
        return false;
    }

    // Reset the sensor data structure
    sensorData = SensorData();
    
    int payloadLength = Xbus::getPayloadLength(xbusData);
    const uint8_t* payload = Xbus::getConstPointerToPayload(xbusData);
    
    int index = 0;
    
    // Parse all data items in the payload
    while (index < payloadLength) {
        if (index + 3 > payloadLength) {
            break; // Not enough bytes for XDI and size
        }
        
        uint16_t xdi = readUint16(payload, index);
        uint8_t size = readUint8(payload, index);
        
        if (index + size > payloadLength) {
            break; // Not enough bytes for the data
        }
        
        switch (xdi) {
            case XDI::PACKET_COUNTER:
                if (size == 2) {
                    sensorData.packetCounter = readUint16(payload, index);
                    sensorData.hasPacketCounter = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;
                
            case XDI::SAMPLE_TIME_FINE:
                if (size == 4) {
                    sensorData.sampleTimeFine = readUint32(payload, index);
                    sensorData.hasSampleTimeFine = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;
                
            case XDI::EULER_ANGLES:
                if (size == 12) {
                    sensorData.eulerAngles.roll = readFloat(payload, index);
                    sensorData.eulerAngles.pitch = readFloat(payload, index);
                    sensorData.eulerAngles.yaw = readFloat(payload, index);
                    sensorData.hasEulerAngles = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;
                
            case XDI::STATUS_WORD:
                if (size == 4) {
                    sensorData.statusWord = readUint32(payload, index);
                    sensorData.hasStatusWord = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;
                
            case XDI::LAT_LON:
                if (size == 12) { // 6 bytes each for lat and lon
                    sensorData.latLon.latitude = readFP1632(payload, index);
                    sensorData.latLon.longitude = readFP1632(payload, index);
                    sensorData.hasLatLon = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;
                
            case XDI::ALTITUDE_ELLIPSOID:
                if (size == 6) { // FP1632 format
                    sensorData.altitudeEllipsoid = readFP1632(payload, index);
                    sensorData.hasAltitudeEllipsoid = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;
                
            case XDI::VELOCITY_XYZ:
                if (size == 18) { // 6 bytes each for X, Y, Z
                    sensorData.velocityXYZ.velX = readFP1632(payload, index);
                    sensorData.velocityXYZ.velY = readFP1632(payload, index);
                    sensorData.velocityXYZ.velZ = readFP1632(payload, index);
                    sensorData.hasVelocityXYZ = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;
                
            default:
                // Unknown XDI, skip it
                index += size;
                break;
        }
    }
    
    return true;
}

std::string XbusParser::formatSensorData(const SensorData& data) {
    std::ostringstream oss;
    
    if (data.hasPacketCounter) {
        oss << "PC=" << data.packetCounter;
    }
    
    if (data.hasSampleTimeFine) {
        if (!oss.str().empty()) oss << ", ";
        oss << "STF=" << data.sampleTimeFine;
    }
    
    if (data.hasEulerAngles) {
        if (!oss.str().empty()) oss << ", ";
        oss << "Euler(R=" << std::fixed << std::setprecision(2) << data.eulerAngles.roll
            << "°, P=" << data.eulerAngles.pitch 
            << "°, Y=" << data.eulerAngles.yaw << "°)";
    }
    
    if (data.hasLatLon) {
        if (!oss.str().empty()) oss << ", ";
        oss << "LatLon(" << std::fixed << std::setprecision(8) << data.latLon.latitude
            << ", " << data.latLon.longitude << ")";
    }
    
    if (data.hasAltitudeEllipsoid) {
        if (!oss.str().empty()) oss << ", ";
        oss << "Alt=" << std::fixed << std::setprecision(3) << data.altitudeEllipsoid << "m";
    }
    
    if (data.hasVelocityXYZ) {
        if (!oss.str().empty()) oss << ", ";
        oss << "Vel(" << std::fixed << std::setprecision(4) << data.velocityXYZ.velX
            << ", " << data.velocityXYZ.velY << ", " << data.velocityXYZ.velZ << ")m/s";
    }
    
    if (data.hasStatusWord) {
        if (!oss.str().empty()) oss << ", ";
        oss << "Status=" << formatStatusWord(data.statusWord);
    }
    
    return oss.str();
}

std::string XbusParser::getXDIName(uint16_t xdi) {
    switch (xdi) {
        case XDI::PACKET_COUNTER: return "PacketCounter";
        case XDI::SAMPLE_TIME_FINE: return "SampleTimeFine";
        case XDI::EULER_ANGLES: return "EulerAngles";
        case XDI::STATUS_WORD: return "StatusWord";
        case XDI::LAT_LON: return "LatLon";
        case XDI::ALTITUDE_ELLIPSOID: return "AltitudeEllipsoid";
        case XDI::VELOCITY_XYZ: return "VelocityXYZ";
        case XDI::QUATERNION: return "Quaternion";
        case XDI::ACCELERATION: return "Acceleration";
        case XDI::RATE_OF_TURN: return "RateOfTurn";
        case XDI::MAGNETIC_FIELD: return "MagneticField";
        default: return "Unknown";
    }
}

std::string XbusParser::formatStatusWord(uint32_t statusWord) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << statusWord;
    
    // Add some basic status interpretation
    if (statusWord & 0x0001) oss << " [SelfTest]";
    if (statusWord & 0x0002) oss << " [FilterValid]";
    if (statusWord & 0x0004) oss << " [GNSSFix]";
    
    return oss.str();
}

bool XbusParser::parseEulerAngles(const uint8_t* xbusData, EulerAngles& angles) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasEulerAngles) {
        angles = sensorData.eulerAngles;
        return true;
    }
    return false;
}

uint32_t XbusParser::parseDeviceId(const uint8_t* xbusData) {
    if (!Xbus::checkPreamble(xbusData)) {
        return 0;
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    if (messageId != XMID_DeviceId) {
        return 0;
    }

    int index = 4;
    return readUint32(xbusData, index);
}

std::string XbusParser::parseFirmwareRevision(const uint8_t* xbusData) {
    if (!Xbus::checkPreamble(xbusData)) {
        return "";
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    if (messageId != XMID_FirmwareRevision) {
        return "";
    }

    int index = 4;
    uint8_t major = readUint8(xbusData, index);
    uint8_t minor = readUint8(xbusData, index);
    uint8_t patch = readUint8(xbusData, index);
    
    std::ostringstream oss;
    oss << static_cast<int>(major) << "." << static_cast<int>(minor) << "." << static_cast<int>(patch);
    return oss.str();
}