#ifndef XBUS_PARSER_H
#define XBUS_PARSER_H

#include "xbus.h"
#include "xbus_message_id.h"
#include <string>
#include <cstdint>
#include <vector>

// Data structure definitions
struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
    
    EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
    EulerAngles(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

struct LatLon {
    double latitude;
    double longitude;
    
    LatLon() : latitude(0.0), longitude(0.0) {}
    LatLon(double lat, double lon) : latitude(lat), longitude(lon) {}
};

struct VelocityXYZ {
    double velX;
    double velY;
    double velZ;
    
    VelocityXYZ() : velX(0.0), velY(0.0), velZ(0.0) {}
    VelocityXYZ(double x, double y, double z) : velX(x), velY(y), velZ(z) {}
};

struct Quaternion {
    float q0;  // w component
    float q1;  // x component
    float q2;  // y component
    float q3;  // z component
    
    Quaternion() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}
    Quaternion(float w, float x, float y, float z) : q0(w), q1(x), q2(y), q3(z) {}
};

struct UtcTime {
    uint32_t nanoseconds;    // Fractional part (nanoseconds)
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t flags;
    
    UtcTime() : nanoseconds(0), year(0), month(0), day(0), hour(0), minute(0), second(0), flags(0) {}
};

struct BarometricPressure {
    uint32_t pressure;  // Pressure in Pa (Pascal)
    
    BarometricPressure() : pressure(0) {}
    BarometricPressure(uint32_t p) : pressure(p) {}
};

struct SensorData {
    bool hasPacketCounter = false;
    bool hasSampleTimeFine = false;
    bool hasEulerAngles = false;
    bool hasStatusWord = false;
    bool hasLatLon = false;
    bool hasAltitudeEllipsoid = false;
    bool hasVelocityXYZ = false;
    bool hasUtcTime = false;
    bool hasQuaternion = false;
    bool hasBarometricPressure = false;
    
    uint16_t packetCounter = 0;
    uint32_t sampleTimeFine = 0;
    EulerAngles eulerAngles;
    uint32_t statusWord = 0;
    LatLon latLon;
    double altitudeEllipsoid = 0.0;
    VelocityXYZ velocityXYZ;
    UtcTime utcTime;
    Quaternion quaternion;
    BarometricPressure barometricPressure;
};

// XDI (Xsens Data Identifier) constants
namespace XDI {
    constexpr uint16_t PACKET_COUNTER = 0x1020;
    constexpr uint16_t SAMPLE_TIME_FINE = 0x1060;
    constexpr uint16_t EULER_ANGLES = 0x2030;
    constexpr uint16_t STATUS_WORD = 0xE020;
    constexpr uint16_t LAT_LON = 0x5042;
    constexpr uint16_t ALTITUDE_ELLIPSOID = 0x5022;
    constexpr uint16_t VELOCITY_XYZ = 0xD012;
    constexpr uint16_t QUATERNION = 0x2010;
    constexpr uint16_t ACCELERATION = 0x4020;
    constexpr uint16_t RATE_OF_TURN = 0x8020;
    constexpr uint16_t MAGNETIC_FIELD = 0xC020;
    constexpr uint16_t UTC_TIME = 0x1010;
    constexpr uint16_t BAROMETRIC_PRESSURE = 0x3010;
}

class XbusParser {
public:
    // Helper functions for reading data from payload
    static uint8_t readUint8(const uint8_t* data, int& index);
    static uint16_t readUint16(const uint8_t* data, int& index);
    static uint32_t readUint32(const uint8_t* data, int& index);
    static float readFloat(const uint8_t* data, int& index);
    static double readFP1632(const uint8_t* data, int& index);  // For FP16.32 format
    
    // Parse specific message types
    static std::string messageToString(const uint8_t* xbusData);
    static bool parseEulerAngles(const uint8_t* xbusData, EulerAngles& angles);
    static bool parseQuaternion(const uint8_t* xbusData, Quaternion& quaternion);
    static bool parseUtcTime(const uint8_t* xbusData, UtcTime& utcTime);
    static bool parseBarometricPressure(const uint8_t* xbusData, BarometricPressure& pressure);
    static uint32_t parseDeviceId(const uint8_t* xbusData);
    static std::string parseFirmwareRevision(const uint8_t* xbusData);
    
    // Enhanced MTData2 parsing
    static bool parseMTData2(const uint8_t* xbusData, SensorData& sensorData);
    static std::string formatSensorData(const SensorData& data);
    
private:
    static std::string getXDIName(uint16_t xdi);
    static std::string formatStatusWord(uint32_t statusWord);
    static std::string formatUtcTime(const UtcTime& utcTime);
    static std::string formatQuaternion(const Quaternion& quaternion);
    static std::string formatBarometricPressure(const BarometricPressure& pressure);
};

#endif // XBUS_PARSER_H