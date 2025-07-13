#include "xbus.h"
#include "xbus_parser.h"
#include "xbus_message_id.h"
#include <iostream>
#include <cstring>
#include <cassert>
#include <cmath>
#include <cstdio>

// Fixed-size arrays instead of std::vector
#define MAX_MESSAGE_SIZE 256
#define MAX_PAYLOAD_SIZE 200

class XbusParserTest {
private:
    int testsPassed = 0;
    int testsTotal = 0;
    
public:
    void runAllTests() {
        printf("=== XBus Parser Test Suite ===\n");
        printf("\n");
        
        testMTData2WithAllComponents();
        testFP1632Conversion();
        testEulerAnglesOnly();
        testLatLonOnly();
        testVelocityOnly();
        testUtcTimeOnly();
        testQuaternionOnly();
        testBarometricPressureOnly();
        
        // IMU sensor tests
        testAccelerationOnly();
        testRateOfTurnOnly();
        testMagneticFieldOnly();
        testAllIMUDataTogether();
        
        // Temperature tests
        testTemperatureOnly();
        testCompleteIMUWithTemperature();
        
        testInvalidMessage();
        testMessageToString();
        testFormatSensorData();
        
        printf("\n");
        printf("=== Test Results ===\n");
        printf("Passed: %d/%d\n", testsPassed, testsTotal);
        
        if (testsPassed == testsTotal) {
            printf("All tests PASSED!\n");
        } else {
            printf("Some tests FAILED!\n");
        }
    }
    
    
private:
    void assertTrue(bool condition, const char* testName) {
        testsTotal++;
        if (condition) {
            testsPassed++;
            printf("[PASS] %s\n", testName);
        } else {
            printf("[FAIL] %s\n", testName);
        }
    }
    
    void assertDoubleEquals(double expected, double actual, double tolerance, const char* testName) {
        testsTotal++;
        bool passed = fabs(expected - actual) <= tolerance;
        if (passed) {
            testsPassed++;
            printf("[PASS] %s (expected: %.12f, actual: %.12f)\n", testName, expected, actual);
        } else {
            printf("[FAIL] %s (expected: %.12f, actual: %.12f, diff: %.12f)\n", 
                   testName, expected, actual, fabs(expected - actual));
        }
    }
    
    void assertFloatEquals(float expected, float actual, float tolerance, const char* testName) {
        testsTotal++;
        bool passed = fabsf(expected - actual) <= tolerance;
        if (passed) {
            testsPassed++;
            printf("[PASS] %s (expected: %.6f, actual: %.6f)\n", testName, 
                   static_cast<double>(expected), static_cast<double>(actual));
        } else {
            printf("[FAIL] %s (expected: %.6f, actual: %.6f, diff: %.6f)\n", 
                   testName, static_cast<double>(expected), static_cast<double>(actual), 
                   static_cast<double>(fabsf(expected - actual)));
        }
    }

    void assertUint32Equals(uint32_t expected, uint32_t actual, const char* testName) {
        testsTotal++;
        if (expected == actual) {
            testsPassed++;
            printf("[PASS] %s (expected: %u, actual: %u)\n", testName, expected, actual);
        } else {
            printf("[FAIL] %s (expected: %u, actual: %u)\n", testName, expected, actual);
        }
    }

    void assertUint16Equals(uint16_t expected, uint16_t actual, const char* testName) {
        testsTotal++;
        if (expected == actual) {
            testsPassed++;
            printf("[PASS] %s (expected: %u, actual: %u)\n", testName, expected, actual);
        } else {
            printf("[FAIL] %s (expected: %u, actual: %u)\n", testName, expected, actual);
        }
    }

    void assertUint8Equals(uint8_t expected, uint8_t actual, const char* testName) {
        testsTotal++;
        if (expected == actual) {
            testsPassed++;
            printf("[PASS] %s (expected: %d, actual: %d)\n", testName, 
                   static_cast<int>(expected), static_cast<int>(actual));
        } else {
            printf("[FAIL] %s (expected: %d, actual: %d)\n", testName, 
                   static_cast<int>(expected), static_cast<int>(actual));
        }
    }

    void doubleToFP1632(double value, uint8_t* result) {
        // Calculate: i = round(value * 2^32)
        int64_t fixedPoint = static_cast<int64_t>(round(value * 4294967296.0));
        
        // Extract 32-bit fractional part and 16-bit integer part
        uint32_t fractionalPart = static_cast<uint32_t>(fixedPoint & 0xFFFFFFFF);
        int16_t integerPart = static_cast<int16_t>((fixedPoint >> 32) & 0xFFFF);
        
        // Add fractional part (big-endian, 4 bytes)
        result[0] = (fractionalPart >> 24) & 0xFF;
        result[1] = (fractionalPart >> 16) & 0xFF;
        result[2] = (fractionalPart >> 8) & 0xFF;
        result[3] = fractionalPart & 0xFF;
        
        // Add integer part (big-endian, 2 bytes)
        result[4] = (integerPart >> 8) & 0xFF;
        result[5] = integerPart & 0xFF;
    }
    
    size_t createMTData2Message(const uint8_t* payload, size_t payloadSize, uint8_t* message) {
        size_t index = 0;
        
        // Create message header
        message[index++] = Xbus::XBUS_PREAMBLE;  // Preamble
        message[index++] = Xbus::XBUS_MASTERDEVICE;  // BID
        message[index++] = XMID_MtData2;  // MID
        message[index++] = static_cast<uint8_t>(payloadSize);  // Length
        
        // Add payload
        memcpy(&message[index], payload, payloadSize);
        index += payloadSize;
        
        // Calculate and add checksum
        uint8_t checksum = 0;
        for (size_t i = 1; i < index; i++) {
            checksum -= message[i];
        }
        message[index++] = checksum;
        
        return index;
    }
    
    void testFP1632Conversion() {
        printf("\n--- Testing FP1632 Conversion ---\n");
        
        // Test data from your screenshot:
        // Latitude: 31.393166223541 -> hex: 64 A6 8A A8 00 1F
        uint8_t latData[] = {0x64, 0xA6, 0x8A, 0xA8, 0x00, 0x1F};
        int index = 0;
        double latitude = XbusParser::readFP1632(latData, index);
        assertDoubleEquals(31.393166223541, latitude, 0.000000000001, "Latitude FP1632 conversion");
        
        // Longitude: 121.229738174938 -> hex: 3A D0 1E FC 00 79
        uint8_t lonData[] = {0x3A, 0xD0, 0x1E, 0xFC, 0x00, 0x79};
        index = 0;
        double longitude = XbusParser::readFP1632(lonData, index);
        assertDoubleEquals(121.229738174938, longitude, 0.000000000001, "Longitude FP1632 conversion");
        
        // Altitude: 56.714969451306 -> hex: B7 0B 3C EB 00 38
        uint8_t altData[] = {0xB7, 0x0B, 0x3C, 0xEB, 0x00, 0x38};
        index = 0;
        double altitude = XbusParser::readFP1632(altData, index);
        assertDoubleEquals(56.714969451306, altitude, 0.0001, "Altitude FP1632 conversion");
    }
    
    void testMTData2WithAllComponents() {
        printf("\n--- Testing MTData2 with All Components ---\n");
        
        // Create payload with all data types from your screenshot
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // PacketCounter (1020): 2826 -> 0B 0A
        uint8_t pcData[] = {0x10, 0x20, 0x02, 0x0B, 0x0A};
        memcpy(&payload[payloadIndex], pcData, sizeof(pcData));
        payloadIndex += sizeof(pcData);
        
        // SampleTimeFine (1060): 12931224 -> 00 C5 50 98
        uint8_t stfData[] = {0x10, 0x60, 0x04, 0x00, 0xC5, 0x50, 0x98};
        memcpy(&payload[payloadIndex], stfData, sizeof(stfData));
        payloadIndex += sizeof(stfData);
        
        // EulerAngles (2030): Roll=179.9332581, Pitch=-1.1505425, Yaw=-2.3420007
        uint8_t eulerData[] = {0x20, 0x30, 0x0C, 
                              0x43, 0x33, 0xEE, 0xEA,  // Roll
                              0xBF, 0x93, 0x44, 0xFA,  // Pitch
                              0xC0, 0x15, 0xE3, 0x57}; // Yaw
        memcpy(&payload[payloadIndex], eulerData, sizeof(eulerData));
        payloadIndex += sizeof(eulerData);
        
        // StatusWord (E020): 00 00 00 02
        uint8_t statusData[] = {0xE0, 0x20, 0x04, 0x00, 0x00, 0x00, 0x02};
        memcpy(&payload[payloadIndex], statusData, sizeof(statusData));
        payloadIndex += sizeof(statusData);
        
        // LatLon (5042): Lat=31.393166223541, Lon=121.229738174938
        uint8_t latLonData[] = {0x50, 0x42, 0x0C,
                               0x64, 0xA6, 0x8A, 0xA8, 0x00, 0x1F,  // Latitude
                               0x3A, 0xD0, 0x1E, 0xFC, 0x00, 0x79}; // Longitude
        memcpy(&payload[payloadIndex], latLonData, sizeof(latLonData));
        payloadIndex += sizeof(latLonData);
        
        // AltitudeEllipsoid (5022): 56.714969451306
        uint8_t altData[] = {0x50, 0x22, 0x06, 0xB7, 0x0B, 0x3C, 0xEB, 0x00, 0x38};
        memcpy(&payload[payloadIndex], altData, sizeof(altData));
        payloadIndex += sizeof(altData);
        
        // VelocityXYZ (D012): X=-0.021542994305, Y=0.013762803748, Z=-0.043488796800
        uint8_t velData[] = {0xD0, 0x12, 0x12,
                            0xFA, 0x7C, 0x28, 0x88, 0xFF, 0xFF,  // velX
                            0x03, 0x85, 0xF5, 0x88, 0x00, 0x00,  // velY
                            0xF4, 0xDD, 0xEB, 0x10, 0xFF, 0xFF}; // velZ
        memcpy(&payload[payloadIndex], velData, sizeof(velData));
        payloadIndex += sizeof(velData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "MTData2 parsing success");
        assertTrue(sensorData.hasPacketCounter, "Has PacketCounter");
        assertTrue(sensorData.hasSampleTimeFine, "Has SampleTimeFine");
        assertTrue(sensorData.hasEulerAngles, "Has EulerAngles");
        assertTrue(sensorData.hasStatusWord, "Has StatusWord");
        assertTrue(sensorData.hasLatLon, "Has LatLon");
        assertTrue(sensorData.hasAltitudeEllipsoid, "Has AltitudeEllipsoid");
        assertTrue(sensorData.hasVelocityXYZ, "Has VelocityXYZ");
        
        // Verify values
        assertTrue(sensorData.packetCounter == 2826, "PacketCounter value");
        assertTrue(sensorData.sampleTimeFine == 12931224, "SampleTimeFine value");
        assertTrue(sensorData.statusWord == 2, "StatusWord value");
        
        assertFloatEquals(179.9332581f, sensorData.eulerAngles.roll, 0.0001f, "Euler Roll");
        assertFloatEquals(-1.1505425f, sensorData.eulerAngles.pitch, 0.0001f, "Euler Pitch");
        assertFloatEquals(-2.3420007f, sensorData.eulerAngles.yaw, 0.0001f, "Euler Yaw");
        
        assertDoubleEquals(31.393166223541, sensorData.latLon.latitude, 0.000000000001, "Latitude");
        assertDoubleEquals(121.229738174938, sensorData.latLon.longitude, 0.000000000001, "Longitude");
        assertDoubleEquals(56.714969451306, sensorData.altitudeEllipsoid, 0.0001, "Altitude");

        assertDoubleEquals(-0.021542994305, sensorData.velocityXYZ.velX, 0.000000000001, "Velocity X");
        assertDoubleEquals(0.013762803748, sensorData.velocityXYZ.velY, 0.000000000001, "Velocity Y");
        assertDoubleEquals(-0.043488796800, sensorData.velocityXYZ.velZ, 0.000000000001, "Velocity Z");
    }
    
    void testEulerAnglesOnly() {
        printf("\n--- Testing Euler Angles Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // EulerAngles (2030): Roll=45.0, Pitch=30.0, Yaw=90.0
        uint8_t eulerData[] = {0x20, 0x30, 0x0C,
                              0x42, 0x34, 0x00, 0x00,  // Roll: 45.0
                              0x41, 0xF0, 0x00, 0x00,  // Pitch: 30.0
                              0x42, 0xB4, 0x00, 0x00}; // Yaw: 90.0
        memcpy(&payload[payloadIndex], eulerData, sizeof(eulerData));
        payloadIndex += sizeof(eulerData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Euler only parsing success");
        assertTrue(sensorData.hasEulerAngles, "Has EulerAngles");
        assertTrue(!sensorData.hasLatLon, "No LatLon");
        assertTrue(!sensorData.hasVelocityXYZ, "No VelocityXYZ");
        
        assertFloatEquals(45.0f, sensorData.eulerAngles.roll, 0.001f, "Euler Roll (45.0)");
        assertFloatEquals(30.0f, sensorData.eulerAngles.pitch, 0.001f, "Euler Pitch (30.0)");
        assertFloatEquals(90.0f, sensorData.eulerAngles.yaw, 0.001f, "Euler Yaw (90.0)");
    }
    
    void testLatLonOnly() {
        printf("\n--- Testing LatLon Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // LatLon (5042): Use simple values that convert cleanly
        // Lat=1.0, Lon=-1.0 for easier verification
        payload[payloadIndex++] = 0x50;  // XDI high byte
        payload[payloadIndex++] = 0x42;  // XDI low byte
        payload[payloadIndex++] = 0x0C;  // Size: 12 bytes
        
        // Generate correct FP16.32 values
        uint8_t lat1[6], lon_1[6];
        doubleToFP1632(1.0, lat1);
        doubleToFP1632(-1.0, lon_1);
        
        memcpy(&payload[payloadIndex], lat1, 6);
        payloadIndex += 6;
        memcpy(&payload[payloadIndex], lon_1, 6);
        payloadIndex += 6;
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "LatLon only parsing success");
        assertTrue(sensorData.hasLatLon, "Has LatLon");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertDoubleEquals(1.0, sensorData.latLon.latitude, 0.000000001, "Latitude (1.0)");
        assertDoubleEquals(-1.0, sensorData.latLon.longitude, 0.000000001, "Longitude (-1.0)");
    }

    void testVelocityOnly() {
        printf("\n--- Testing Velocity Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // VelocityXYZ (D012): Use simple values X=0.1, Y=0.2, Z=0.3
        payload[payloadIndex++] = 0xD0;  // XDI high byte
        payload[payloadIndex++] = 0x12;  // XDI low byte
        payload[payloadIndex++] = 0x12;  // Size: 18 bytes
        
        // Generate correct FP16.32 values
        uint8_t vel01[6], vel02[6], vel03[6];
        doubleToFP1632(0.1, vel01);
        doubleToFP1632(0.2, vel02);
        doubleToFP1632(0.3, vel03);
        
        memcpy(&payload[payloadIndex], vel01, 6);
        payloadIndex += 6;
        memcpy(&payload[payloadIndex], vel02, 6);
        payloadIndex += 6;
        memcpy(&payload[payloadIndex], vel03, 6);
        payloadIndex += 6;
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Velocity only parsing success");
        assertTrue(sensorData.hasVelocityXYZ, "Has VelocityXYZ");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertDoubleEquals(0.1, sensorData.velocityXYZ.velX, 0.000000001, "Velocity X (0.1)");
        assertDoubleEquals(0.2, sensorData.velocityXYZ.velY, 0.000000001, "Velocity Y (0.2)");
        assertDoubleEquals(0.3, sensorData.velocityXYZ.velZ, 0.000000001, "Velocity Z (0.3)");
    }

    void testUtcTimeOnly() {
        printf("\n--- Testing UTC Time Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // UTC Time test data: 10 10 0C 2C A8 4D 3C 07 E9 07 0D 09 15 22 00
        uint8_t utcData[] = {0x10, 0x10, 0x0C, 
                            0x2C, 0xA8, 0x4D, 0x3C,  // nanoseconds: 749227324
                            0x07, 0xE9,              // year: 2025
                            0x07,                    // month: 7
                            0x0D,                    // day: 13
                            0x09,                    // hour: 9
                            0x15,                    // minute: 21
                            0x22,                    // second: 34
                            0x00};                   // flags: 0
        memcpy(&payload[payloadIndex], utcData, sizeof(utcData));
        payloadIndex += sizeof(utcData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "UTC Time parsing success");
        assertTrue(sensorData.hasUtcTime, "Has UtcTime");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertUint32Equals(749227324, sensorData.utcTime.nanoseconds, "UTC nanoseconds");
        assertUint16Equals(2025, sensorData.utcTime.year, "UTC year");
        assertUint8Equals(7, sensorData.utcTime.month, "UTC month");
        assertUint8Equals(13, sensorData.utcTime.day, "UTC day");
        assertUint8Equals(9, sensorData.utcTime.hour, "UTC hour");
        assertUint8Equals(21, sensorData.utcTime.minute, "UTC minute");
        assertUint8Equals(34, sensorData.utcTime.second, "UTC second");
        assertUint8Equals(0, sensorData.utcTime.flags, "UTC flags");
    }

    void testQuaternionOnly() {
        printf("\n--- Testing Quaternion Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Quaternion test data: 20 10 10 3F 7F FE F3 BA 9C 8E C3 3A FD 24 45 3B AA 72 59
        uint8_t quatData[] = {0x20, 0x10, 0x10,
                             0x3F, 0x7F, 0xFE, 0xF3,  // q0: 0.9999840
                             0xBA, 0x9C, 0x8E, 0xC3,  // q1: -0.0011944
                             0x3A, 0xFD, 0x24, 0x45,  // q2: 0.0019313
                             0x3B, 0xAA, 0x72, 0x59}; // q3: 0.0052016
        memcpy(&payload[payloadIndex], quatData, sizeof(quatData));
        payloadIndex += sizeof(quatData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Quaternion parsing success");
        assertTrue(sensorData.hasQuaternion, "Has Quaternion");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertFloatEquals(0.9999840f, sensorData.quaternion.q0, 0.0000001f, "Quaternion q0");
        assertFloatEquals(-0.0011944f, sensorData.quaternion.q1, 0.0000001f, "Quaternion q1");
        assertFloatEquals(0.0019313f, sensorData.quaternion.q2, 0.0000001f, "Quaternion q2");
        assertFloatEquals(0.0052016f, sensorData.quaternion.q3, 0.0000001f, "Quaternion q3");
    }

    void testBarometricPressureOnly() {
        printf("\n--- Testing Barometric Pressure Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Barometric pressure test data: 30 10 04 00 01 87 A4
        uint8_t baroData[] = {0x30, 0x10, 0x04,
                             0x00, 0x01, 0x87, 0xA4}; // pressure: 100260
        memcpy(&payload[payloadIndex], baroData, sizeof(baroData));
        payloadIndex += sizeof(baroData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Barometric pressure parsing success");
        assertTrue(sensorData.hasBarometricPressure, "Has BarometricPressure");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertUint32Equals(100260, sensorData.barometricPressure.pressure, "Barometric pressure value");
    }

    void testAccelerationOnly() {
        printf("\n--- Testing Acceleration Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Acceleration test data: 40 20 0C BC DF C3 F0 BD 32 77 7B 41 1C CD 9B
        // Values: X=-0.0273151, Y=-0.0435710, Z=9.8001966
        uint8_t accData[] = {0x40, 0x20, 0x0C,
                            0xBC, 0xDF, 0xC3, 0xF0,  // accX: -0.0273151
                            0xBD, 0x32, 0x77, 0x7B,  // accY: -0.0435710
                            0x41, 0x1C, 0xCD, 0x9B}; // accZ: 9.8001966
        memcpy(&payload[payloadIndex], accData, sizeof(accData));
        payloadIndex += sizeof(accData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Acceleration parsing success");
        assertTrue(sensorData.hasAcceleration, "Has Acceleration");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        assertTrue(!sensorData.hasRateOfTurn, "No RateOfTurn");
        assertTrue(!sensorData.hasMagneticField, "No MagneticField");
        
        assertFloatEquals(-0.0273151f, sensorData.acceleration.accX, 0.0000001f, "Acceleration X");
        assertFloatEquals(-0.0435710f, sensorData.acceleration.accY, 0.0000001f, "Acceleration Y");
        assertFloatEquals(9.8001966f, sensorData.acceleration.accZ, 0.0000001f, "Acceleration Z");
        
        // Test individual parse function
        AccelerationXYZ acceleration;
        bool parseSuccess = XbusParser::parseAcceleration(message, acceleration);
        assertTrue(parseSuccess, "parseAcceleration function success");
        assertFloatEquals(-0.0273151f, acceleration.accX, 0.0000001f, "parseAcceleration X");
        assertFloatEquals(-0.0435710f, acceleration.accY, 0.0000001f, "parseAcceleration Y");
        assertFloatEquals(9.8001966f, acceleration.accZ, 0.0000001f, "parseAcceleration Z");
    }
    
    void testRateOfTurnOnly() {
        printf("\n--- Testing Rate of Turn Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Rate of turn test data: 80 20 0C 3B EE B2 40 3B 29 49 81 3B AC D3 C0
        // Values: X=0.0072844, Y=0.0025831, Z=0.0052743
        uint8_t rotData[] = {0x80, 0x20, 0x0C,
                            0x3B, 0xEE, 0xB2, 0x40,  // gyrX: 0.0072844
                            0x3B, 0x29, 0x49, 0x81,  // gyrY: 0.0025831
                            0x3B, 0xAC, 0xD3, 0xC0}; // gyrZ: 0.0052743
        memcpy(&payload[payloadIndex], rotData, sizeof(rotData));
        payloadIndex += sizeof(rotData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Rate of turn parsing success");
        assertTrue(sensorData.hasRateOfTurn, "Has RateOfTurn");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        assertTrue(!sensorData.hasAcceleration, "No Acceleration");
        assertTrue(!sensorData.hasMagneticField, "No MagneticField");
        
        assertFloatEquals(0.0072844f, sensorData.rateOfTurn.gyrX, 0.0000001f, "Rate of turn X");
        assertFloatEquals(0.0025831f, sensorData.rateOfTurn.gyrY, 0.0000001f, "Rate of turn Y");
        assertFloatEquals(0.0052743f, sensorData.rateOfTurn.gyrZ, 0.0000001f, "Rate of turn Z");
        
        // Test individual parse function
        RateOfTurnXYZ rateOfTurn;
        bool parseSuccess = XbusParser::parseRateOfTurn(message, rateOfTurn);
        assertTrue(parseSuccess, "parseRateOfTurn function success");
        assertFloatEquals(0.0072844f, rateOfTurn.gyrX, 0.0000001f, "parseRateOfTurn X");
        assertFloatEquals(0.0025831f, rateOfTurn.gyrY, 0.0000001f, "parseRateOfTurn Y");
        assertFloatEquals(0.0052743f, rateOfTurn.gyrZ, 0.0000001f, "parseRateOfTurn Z");
    }
    
    void testMagneticFieldOnly() {
        printf("\n--- Testing Magnetic Field Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Magnetic field test data: C0 20 0C BE BB F8 D0 BE D3 69 60 BF 4D B3 B4
        // Values: X=-0.3671327, Y=-0.4129133, Z=-0.8035233
        uint8_t magData[] = {0xC0, 0x20, 0x0C,
                            0xBE, 0xBB, 0xF8, 0xD0,  // magX: -0.3671327
                            0xBE, 0xD3, 0x69, 0x60,  // magY: -0.4129133
                            0xBF, 0x4D, 0xB3, 0xB4}; // magZ: -0.8035233
        memcpy(&payload[payloadIndex], magData, sizeof(magData));
        payloadIndex += sizeof(magData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Magnetic field parsing success");
        assertTrue(sensorData.hasMagneticField, "Has MagneticField");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        assertTrue(!sensorData.hasAcceleration, "No Acceleration");
        assertTrue(!sensorData.hasRateOfTurn, "No RateOfTurn");
        
        assertFloatEquals(-0.3671327f, sensorData.magneticField.magX, 0.0000001f, "Magnetic field X");
        assertFloatEquals(-0.4129133f, sensorData.magneticField.magY, 0.0000001f, "Magnetic field Y");
        assertFloatEquals(-0.8035233f, sensorData.magneticField.magZ, 0.0000001f, "Magnetic field Z");
        
        // Test individual parse function
        MagneticFieldXYZ magneticField;
        bool parseSuccess = XbusParser::parseMagneticField(message, magneticField);
        assertTrue(parseSuccess, "parseMagneticField function success");
        assertFloatEquals(-0.3671327f, magneticField.magX, 0.0000001f, "parseMagneticField X");
        assertFloatEquals(-0.4129133f, magneticField.magY, 0.0000001f, "parseMagneticField Y");
        assertFloatEquals(-0.8035233f, magneticField.magZ, 0.0000001f, "parseMagneticField Z");
    }
    
    void testAllIMUDataTogether() {
        printf("\n--- Testing All IMU Data Together ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Acceleration data: 40 20 0C BC DF C3 F0 BD 32 77 7B 41 1C CD 9B
        uint8_t accData[] = {0x40, 0x20, 0x0C,
                            0xBC, 0xDF, 0xC3, 0xF0,  // accX: -0.0273151
                            0xBD, 0x32, 0x77, 0x7B,  // accY: -0.0435710
                            0x41, 0x1C, 0xCD, 0x9B}; // accZ: 9.8001966
        memcpy(&payload[payloadIndex], accData, sizeof(accData));
        payloadIndex += sizeof(accData);
        
        // Rate of turn data: 80 20 0C 3B EE B2 40 3B 29 49 81 3B AC D3 C0
        uint8_t rotData[] = {0x80, 0x20, 0x0C,
                            0x3B, 0xEE, 0xB2, 0x40,  // gyrX: 0.0072844
                            0x3B, 0x29, 0x49, 0x81,  // gyrY: 0.0025831
                            0x3B, 0xAC, 0xD3, 0xC0}; // gyrZ: 0.0052743
        memcpy(&payload[payloadIndex], rotData, sizeof(rotData));
        payloadIndex += sizeof(rotData);
        
        // Magnetic field data: C0 20 0C BE BB F8 D0 BE D3 69 60 BF 4D B3 B4
        uint8_t magData[] = {0xC0, 0x20, 0x0C,
                            0xBE, 0xBB, 0xF8, 0xD0,  // magX: -0.3671327
                            0xBE, 0xD3, 0x69, 0x60,  // magY: -0.4129133
                            0xBF, 0x4D, 0xB3, 0xB4}; // magZ: -0.8035233
        memcpy(&payload[payloadIndex], magData, sizeof(magData));
        payloadIndex += sizeof(magData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "All IMU data parsing success");
        assertTrue(sensorData.hasAcceleration, "Has Acceleration");
        assertTrue(sensorData.hasRateOfTurn, "Has RateOfTurn");
        assertTrue(sensorData.hasMagneticField, "Has MagneticField");
        
        // Verify acceleration values
        assertFloatEquals(-0.0273151f, sensorData.acceleration.accX, 0.0000001f, "Combined Acceleration X");
        assertFloatEquals(-0.0435710f, sensorData.acceleration.accY, 0.0000001f, "Combined Acceleration Y");
        assertFloatEquals(9.8001966f, sensorData.acceleration.accZ, 0.0000001f, "Combined Acceleration Z");
        
        // Verify rate of turn values
        assertFloatEquals(0.0072844f, sensorData.rateOfTurn.gyrX, 0.0000001f, "Combined Rate of turn X");
        assertFloatEquals(0.0025831f, sensorData.rateOfTurn.gyrY, 0.0000001f, "Combined Rate of turn Y");
        assertFloatEquals(0.0052743f, sensorData.rateOfTurn.gyrZ, 0.0000001f, "Combined Rate of turn Z");
        
        // Verify magnetic field values
        assertFloatEquals(-0.3671327f, sensorData.magneticField.magX, 0.0000001f, "Combined Magnetic field X");
        assertFloatEquals(-0.4129133f, sensorData.magneticField.magY, 0.0000001f, "Combined Magnetic field Y");
        assertFloatEquals(-0.8035233f, sensorData.magneticField.magZ, 0.0000001f, "Combined Magnetic field Z");
        
        // Test formatted output includes all IMU data
        char output[MAX_SENSOR_DATA_STRING_LEN];
        bool formatSuccess = XbusParser::formatSensorData(sensorData, output, sizeof(output));
        assertTrue(formatSuccess, "IMU data formatting success");
        printf("Combined IMU data: %s\n", output);
        
        // Check that the formatted string contains acceleration, rate of turn, and magnetic field data
        assertTrue(strstr(output, "Acc=") != nullptr, "Formatted output contains acceleration");
        assertTrue(strstr(output, "RoT=") != nullptr, "Formatted output contains rate of turn");
        assertTrue(strstr(output, "Mag=") != nullptr, "Formatted output contains magnetic field");
    }

    void testTemperatureOnly() {
        printf("\n--- Testing Temperature Only ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Temperature test data: 08 10 04 42 13 98 00
        // Value: 36.8984375°C
        uint8_t tempData[] = {0x08, 0x10, 0x04,
                             0x42, 0x13, 0x98, 0x00}; // temperature: 36.8984375
        memcpy(&payload[payloadIndex], tempData, sizeof(tempData));
        payloadIndex += sizeof(tempData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Temperature parsing success");
        assertTrue(sensorData.hasTemperature, "Has Temperature");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        assertTrue(!sensorData.hasAcceleration, "No Acceleration");
        assertTrue(!sensorData.hasRateOfTurn, "No RateOfTurn");
        assertTrue(!sensorData.hasMagneticField, "No MagneticField");
        
        assertFloatEquals(36.8984375f, sensorData.temperature.temperature, 0.0000001f, "Temperature value");
        
        // Test individual parse function
        Temperature temperature;
        bool parseSuccess = XbusParser::parseTemperature(message, temperature);
        assertTrue(parseSuccess, "parseTemperature function success");
        assertFloatEquals(36.8984375f, temperature.temperature, 0.0000001f, "parseTemperature value");
        
        // Test formatted output
        char output[MAX_SENSOR_DATA_STRING_LEN];
        bool formatSuccess = XbusParser::formatSensorData(sensorData, output, sizeof(output));
        assertTrue(formatSuccess, "Temperature formatting success");
        printf("Temperature data: %s\n", output);
        
        // Check that the formatted string contains temperature data
        assertTrue(strstr(output, "Temp=") != nullptr, "Formatted output contains temperature");
        assertTrue(strstr(output, "°C") != nullptr, "Formatted output contains temperature unit");
    }
    
    void testCompleteIMUWithTemperature() {
        printf("\n--- Testing Complete IMU Data with Temperature ---\n");
        
        uint8_t payload[MAX_PAYLOAD_SIZE];
        size_t payloadIndex = 0;
        
        // Acceleration data: 40 20 0C BC DF C3 F0 BD 32 77 7B 41 1C CD 9B
        uint8_t accData[] = {0x40, 0x20, 0x0C,
                            0xBC, 0xDF, 0xC3, 0xF0,  // accX: -0.0273151
                            0xBD, 0x32, 0x77, 0x7B,  // accY: -0.0435710
                            0x41, 0x1C, 0xCD, 0x9B}; // accZ: 9.8001966
        memcpy(&payload[payloadIndex], accData, sizeof(accData));
        payloadIndex += sizeof(accData);
        
        // Rate of turn data: 80 20 0C 3B EE B2 40 3B 29 49 81 3B AC D3 C0
        uint8_t rotData[] = {0x80, 0x20, 0x0C,
                            0x3B, 0xEE, 0xB2, 0x40,  // gyrX: 0.0072844
                            0x3B, 0x29, 0x49, 0x81,  // gyrY: 0.0025831
                            0x3B, 0xAC, 0xD3, 0xC0}; // gyrZ: 0.0052743
        memcpy(&payload[payloadIndex], rotData, sizeof(rotData));
        payloadIndex += sizeof(rotData);
        
        // Magnetic field data: C0 20 0C BE BB F8 D0 BE D3 69 60 BF 4D B3 B4
        uint8_t magData[] = {0xC0, 0x20, 0x0C,
                            0xBE, 0xBB, 0xF8, 0xD0,  // magX: -0.3671327
                            0xBE, 0xD3, 0x69, 0x60,  // magY: -0.4129133
                            0xBF, 0x4D, 0xB3, 0xB4}; // magZ: -0.8035233
        memcpy(&payload[payloadIndex], magData, sizeof(magData));
        payloadIndex += sizeof(magData);
        
        // Temperature data: 08 10 04 42 13 98 00
        uint8_t tempData[] = {0x08, 0x10, 0x04,
                             0x42, 0x13, 0x98, 0x00}; // temperature: 36.8984375
        memcpy(&payload[payloadIndex], tempData, sizeof(tempData));
        payloadIndex += sizeof(tempData);
        
        uint8_t message[MAX_MESSAGE_SIZE];
        size_t messageSize = createMTData2Message(payload, payloadIndex, message);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message, sensorData);
        
        assertTrue(success, "Complete IMU with temperature parsing success");
        assertTrue(sensorData.hasAcceleration, "Has Acceleration");
        assertTrue(sensorData.hasRateOfTurn, "Has RateOfTurn");
        assertTrue(sensorData.hasMagneticField, "Has MagneticField");
        assertTrue(sensorData.hasTemperature, "Has Temperature");
        
        // Verify acceleration values
        assertFloatEquals(-0.0273151f, sensorData.acceleration.accX, 0.0000001f, "Complete Acceleration X");
        assertFloatEquals(-0.0435710f, sensorData.acceleration.accY, 0.0000001f, "Complete Acceleration Y");
        assertFloatEquals(9.8001966f, sensorData.acceleration.accZ, 0.0000001f, "Complete Acceleration Z");
        
        // Verify rate of turn values
        assertFloatEquals(0.0072844f, sensorData.rateOfTurn.gyrX, 0.0000001f, "Complete Rate of turn X");
        assertFloatEquals(0.0025831f, sensorData.rateOfTurn.gyrY, 0.0000001f, "Complete Rate of turn Y");
        assertFloatEquals(0.0052743f, sensorData.rateOfTurn.gyrZ, 0.0000001f, "Complete Rate of turn Z");
        
        // Verify magnetic field values
        assertFloatEquals(-0.3671327f, sensorData.magneticField.magX, 0.0000001f, "Complete Magnetic field X");
        assertFloatEquals(-0.4129133f, sensorData.magneticField.magY, 0.0000001f, "Complete Magnetic field Y");
        assertFloatEquals(-0.8035233f, sensorData.magneticField.magZ, 0.0000001f, "Complete Magnetic field Z");
        
        // Verify temperature value
        assertFloatEquals(36.8984375f, sensorData.temperature.temperature, 0.0000001f, "Complete Temperature");
        
        // Test formatted output includes all IMU data and temperature
        char output[MAX_SENSOR_DATA_STRING_LEN];
        bool formatSuccess = XbusParser::formatSensorData(sensorData, output, sizeof(output));
        assertTrue(formatSuccess, "Complete IMU with temperature formatting success");
        printf("Complete IMU with temperature: %s\n", output);
        
        // Check that the formatted string contains all data types
        assertTrue(strstr(output, "Acc=") != nullptr, "Formatted output contains acceleration");
        assertTrue(strstr(output, "RoT=") != nullptr, "Formatted output contains rate of turn");
        assertTrue(strstr(output, "Mag=") != nullptr, "Formatted output contains magnetic field");
        assertTrue(strstr(output, "Temp=") != nullptr, "Formatted output contains temperature");
        assertTrue(strstr(output, "°C") != nullptr, "Formatted output contains temperature unit");
    }

    void testInvalidMessage() {
        printf("\n--- Testing Invalid Message ---\n");
        
        // Test with invalid preamble
        uint8_t invalidMessage[] = {0xFF, 0xFF, 0x36, 0x00, 0x00};
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(invalidMessage, sensorData);
        assertTrue(!success, "Invalid preamble rejection");
        
        // Test with wrong message ID
        uint8_t wrongMsgId[] = {0xFA, 0xFF, 0x01, 0x00, 0x00};
        success = XbusParser::parseMTData2(wrongMsgId, sensorData);
        assertTrue(!success, "Wrong message ID rejection");
    }

    void testMessageToString() {
        printf("\n--- Testing messageToString ---\n");
        
        // Test DeviceId message
        uint8_t deviceIdMsg[] = {0xFA, 0xFF, 0x01, 0x04, 0x12, 0x34, 0x56, 0x78, 0x95};
        char output[MAX_MESSAGE_STRING_LEN];
        bool success = XbusParser::messageToString(deviceIdMsg, output, sizeof(output));
        assertTrue(success, "DeviceId messageToString success");
        
        // Test Wakeup message
        uint8_t wakeupMsg[] = {0xFA, 0xFF, 0x3E, 0x00, 0xC2};
        success = XbusParser::messageToString(wakeupMsg, output, sizeof(output));
        assertTrue(success, "Wakeup messageToString success");
        printf("Wakeup message: %s\n", output);
        
        // Test invalid message
        uint8_t invalidMsg[] = {0xFF, 0xFF, 0x36, 0x00, 0x00};
        success = XbusParser::messageToString(invalidMsg, output, sizeof(output));
        assertTrue(!success, "Invalid messageToString rejection");
    }

    void testFormatSensorData() {
        printf("\n--- Testing formatSensorData ---\n");
        
        // Create sensor data with some values
        SensorData sensorData;
        sensorData.hasPacketCounter = true;
        sensorData.packetCounter = 1234;
        sensorData.hasEulerAngles = true;
        sensorData.eulerAngles.roll = 45.5f;
        sensorData.eulerAngles.pitch = -30.2f;
        sensorData.eulerAngles.yaw = 180.0f;
        
        char output[MAX_SENSOR_DATA_STRING_LEN];
        bool success = XbusParser::formatSensorData(sensorData, output, sizeof(output));
        assertTrue(success, "formatSensorData success");
        printf("Formatted sensor data: %s\n", output);
        
        // Test empty sensor data
        SensorData emptySensorData;
        success = XbusParser::formatSensorData(emptySensorData, output, sizeof(output));
        assertTrue(success, "Empty formatSensorData success");
        printf("Empty sensor data: '%s'\n", output);
    }
};

int main() {
    XbusParserTest test;
    test.runAllTests();
    return 0;
}