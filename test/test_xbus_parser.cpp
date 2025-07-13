#include "xbus.h"
#include "xbus_parser.h"
#include "xbus_message_id.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <cassert>
#include <cmath>

class XbusParserTest {
private:
    int testsPassed = 0;
    int testsTotal = 0;
    
public:
    void runAllTests() {
        std::cout << "=== XBus Parser Test Suite ===" << std::endl;
        std::cout << std::endl;
        
        testMTData2WithAllComponents();
        testFP1632Conversion();
        testEulerAnglesOnly();
        testLatLonOnly();
        testVelocityOnly();
        testUtcTimeOnly();
        testQuaternionOnly();
        testBarometricPressureOnly();
        testInvalidMessage();
        
        std::cout << std::endl;
        std::cout << "=== Test Results ===" << std::endl;
        std::cout << "Passed: " << testsPassed << "/" << testsTotal << std::endl;
        
        if (testsPassed == testsTotal) {
            std::cout << "All tests PASSED!" << std::endl;
        } else {
            std::cout << "Some tests FAILED!" << std::endl;
        }
    }
    
private:
    void assertTrue(bool condition, const std::string& testName) {
        testsTotal++;
        if (condition) {
            testsPassed++;
            std::cout << "[PASS] " << testName << std::endl;
        } else {
            std::cout << "[FAIL] " << testName << std::endl;
        }
    }
    
    void assertDoubleEquals(double expected, double actual, double tolerance, const std::string& testName) {
        testsTotal++;
        bool passed = std::abs(expected - actual) <= tolerance;
        if (passed) {
            testsPassed++;
            std::cout << "[PASS] " << testName << " (expected: " << expected << ", actual: " << actual << ")" << std::endl;
        } else {
            std::cout << "[FAIL] " << testName << " (expected: " << expected << ", actual: " << actual << ", diff: " << std::abs(expected - actual) << ")" << std::endl;
        }
    }
    
    void assertFloatEquals(float expected, float actual, float tolerance, const std::string& testName) {
        testsTotal++;
        bool passed = std::abs(expected - actual) <= tolerance;
        if (passed) {
            testsPassed++;
            std::cout << "[PASS] " << testName << " (expected: " << expected << ", actual: " << actual << ")" << std::endl;
        } else {
            std::cout << "[FAIL] " << testName << " (expected: " << expected << ", actual: " << actual << ", diff: " << std::abs(expected - actual) << ")" << std::endl;
        }
    }

    void assertUint32Equals(uint32_t expected, uint32_t actual, const std::string& testName) {
        testsTotal++;
        if (expected == actual) {
            testsPassed++;
            std::cout << "[PASS] " << testName << " (expected: " << expected << ", actual: " << actual << ")" << std::endl;
        } else {
            std::cout << "[FAIL] " << testName << " (expected: " << expected << ", actual: " << actual << ")" << std::endl;
        }
    }

    void assertUint16Equals(uint16_t expected, uint16_t actual, const std::string& testName) {
        testsTotal++;
        if (expected == actual) {
            testsPassed++;
            std::cout << "[PASS] " << testName << " (expected: " << expected << ", actual: " << actual << ")" << std::endl;
        } else {
            std::cout << "[FAIL] " << testName << " (expected: " << expected << ", actual: " << actual << ")" << std::endl;
        }
    }

    void assertUint8Equals(uint8_t expected, uint8_t actual, const std::string& testName) {
        testsTotal++;
        if (expected == actual) {
            testsPassed++;
            std::cout << "[PASS] " << testName << " (expected: " << static_cast<int>(expected) << ", actual: " << static_cast<int>(actual) << ")" << std::endl;
        } else {
            std::cout << "[FAIL] " << testName << " (expected: " << static_cast<int>(expected) << ", actual: " << static_cast<int>(actual) << ")" << std::endl;
        }
    }

    std::vector<uint8_t> doubleToFP1632(double value) {
        // Calculate: i = round(value * 2^32)
        int64_t fixedPoint = static_cast<int64_t>(std::round(value * 4294967296.0));
        
        // Extract 32-bit fractional part and 16-bit integer part
        uint32_t fractionalPart = static_cast<uint32_t>(fixedPoint & 0xFFFFFFFF);
        int16_t integerPart = static_cast<int16_t>((fixedPoint >> 32) & 0xFFFF);
        
        std::vector<uint8_t> result;
        
        // Add fractional part (big-endian, 4 bytes)
        result.push_back((fractionalPart >> 24) & 0xFF);
        result.push_back((fractionalPart >> 16) & 0xFF);
        result.push_back((fractionalPart >> 8) & 0xFF);
        result.push_back(fractionalPart & 0xFF);
        
        // Add integer part (big-endian, 2 bytes)
        result.push_back((integerPart >> 8) & 0xFF);
        result.push_back(integerPart & 0xFF);
        
        return result;
    }
    
    std::vector<uint8_t> createMTData2Message(const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> message;
        
        // Create message header
        message.push_back(Xbus::XBUS_PREAMBLE);  // Preamble
        message.push_back(Xbus::XBUS_MASTERDEVICE);  // BID
        message.push_back(XMID_MtData2);  // MID
        message.push_back(static_cast<uint8_t>(payload.size()));  // Length
        
        // Add payload
        message.insert(message.end(), payload.begin(), payload.end());
        
        // Calculate and add checksum
        uint8_t checksum = 0;
        for (size_t i = 1; i < message.size(); i++) {
            checksum -= message[i];
        }
        message.push_back(checksum);
        
        return message;
    }
    
    void testFP1632Conversion() {
        std::cout << std::endl << "--- Testing FP1632 Conversion ---" << std::endl;
        
        // Test data from your screenshot:
        // Latitude: 31.393166223541 -> hex: 64 A6 8A A8 00 1F
        std::vector<uint8_t> latData = {0x64, 0xA6, 0x8A, 0xA8, 0x00, 0x1F};
        int index = 0;
        double latitude = XbusParser::readFP1632(latData.data(), index);
        assertDoubleEquals(31.393166223541, latitude, 0.000000000001, "Latitude FP1632 conversion");
        
        // Longitude: 121.229738174938 -> hex: 3A D0 1E FC 00 79
        std::vector<uint8_t> lonData = {0x3A, 0xD0, 0x1E, 0xFC, 0x00, 0x79};
        index = 0;
        double longitude = XbusParser::readFP1632(lonData.data(), index);
        assertDoubleEquals(121.229738174938, longitude, 0.000000000001, "Longitude FP1632 conversion");
        
        // Altitude: 56.714969451306 -> hex: B7 0B 3C EB 00 38
        std::vector<uint8_t> altData = {0xB7, 0x0B, 0x3C, 0xEB, 0x00, 0x38};
        index = 0;
        double altitude = XbusParser::readFP1632(altData.data(), index);
        assertDoubleEquals(56.714969451306, altitude, 0.0001, "Altitude FP1632 conversion");
    }
    
    void testMTData2WithAllComponents() {
        std::cout << std::endl << "--- Testing MTData2 with All Components ---" << std::endl;
        
        // Create payload with all data types from your screenshot
        std::vector<uint8_t> payload;
        
        // PacketCounter (1020): 2826 -> 0B 0A
        payload.insert(payload.end(), {0x10, 0x20, 0x02, 0x0B, 0x0A});
        
        // SampleTimeFine (1060): 12931224 -> 00 C5 50 98
        payload.insert(payload.end(), {0x10, 0x60, 0x04, 0x00, 0xC5, 0x50, 0x98});
        
        // EulerAngles (2030): Roll=179.9332581, Pitch=-1.1505425, Yaw=-2.3420007
        // Roll: 43 33 EE EA, Pitch: BF 93 44 FA, Yaw: C0 15 E3 57
        payload.insert(payload.end(), {0x20, 0x30, 0x0C, 
                                      0x43, 0x33, 0xEE, 0xEA,  // Roll
                                      0xBF, 0x93, 0x44, 0xFA,  // Pitch
                                      0xC0, 0x15, 0xE3, 0x57}); // Yaw
        
        // StatusWord (E020): 00 00 00 02
        payload.insert(payload.end(), {0xE0, 0x20, 0x04, 0x00, 0x00, 0x00, 0x02});
        
        // LatLon (5042): Lat=31.393166223541, Lon=121.229738174938
        payload.insert(payload.end(), {0x50, 0x42, 0x0C,
                                      0x64, 0xA6, 0x8A, 0xA8, 0x00, 0x1F,  // Latitude
                                      0x3A, 0xD0, 0x1E, 0xFC, 0x00, 0x79}); // Longitude
        
        // AltitudeEllipsoid (5022): 56.714969451306
        payload.insert(payload.end(), {0x50, 0x22, 0x06, 0xB7, 0x0B, 0x3C, 0xEB, 0x00, 0x38});
        
        // VelocityXYZ (D012): X=-0.021542994305, Y=0.013762803748, Z=-0.043488796800
        payload.insert(payload.end(), {0xD0, 0x12, 0x12,
                                      0xFA, 0x7C, 0x28, 0x88, 0xFF, 0xFF,  // velX
                                      0x03, 0x85, 0xF5, 0x88, 0x00, 0x00,  // velY
                                      0xF4, 0xDD, 0xEB, 0x10, 0xFF, 0xFF}); // velZ
        
        std::vector<uint8_t> message = createMTData2Message(payload);
        
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message.data(), sensorData);
        
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
        std::cout << std::endl << "--- Testing Euler Angles Only ---" << std::endl;
        
        std::vector<uint8_t> payload;
        // EulerAngles (2030): Roll=45.0, Pitch=30.0, Yaw=90.0
        payload.insert(payload.end(), {0x20, 0x30, 0x0C,
                                      0x42, 0x34, 0x00, 0x00,  // Roll: 45.0
                                      0x41, 0xF0, 0x00, 0x00,  // Pitch: 30.0
                                      0x42, 0xB4, 0x00, 0x00}); // Yaw: 90.0
        
        std::vector<uint8_t> message = createMTData2Message(payload);
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message.data(), sensorData);
        
        assertTrue(success, "Euler only parsing success");
        assertTrue(sensorData.hasEulerAngles, "Has EulerAngles");
        assertTrue(!sensorData.hasLatLon, "No LatLon");
        assertTrue(!sensorData.hasVelocityXYZ, "No VelocityXYZ");
        
        assertFloatEquals(45.0f, sensorData.eulerAngles.roll, 0.001f, "Euler Roll (45.0)");
        assertFloatEquals(30.0f, sensorData.eulerAngles.pitch, 0.001f, "Euler Pitch (30.0)");
        assertFloatEquals(90.0f, sensorData.eulerAngles.yaw, 0.001f, "Euler Yaw (90.0)");
    }
    
    void testLatLonOnly() {
        std::cout << std::endl << "--- Testing LatLon Only ---" << std::endl;
        
        std::vector<uint8_t> payload;
        // LatLon (5042): Use simple values that convert cleanly
        // Lat=1.0, Lon=-1.0 for easier verification
        payload.insert(payload.end(), {0x50, 0x42, 0x0C});
        
        // Generate correct FP16.32 values
        std::vector<uint8_t> lat1 = doubleToFP1632(1.0);
        std::vector<uint8_t> lon_1 = doubleToFP1632(-1.0);
        
        payload.insert(payload.end(), lat1.begin(), lat1.end());
        payload.insert(payload.end(), lon_1.begin(), lon_1.end());
        
        std::vector<uint8_t> message = createMTData2Message(payload);
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message.data(), sensorData);
        
        assertTrue(success, "LatLon only parsing success");
        assertTrue(sensorData.hasLatLon, "Has LatLon");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertDoubleEquals(1.0, sensorData.latLon.latitude, 0.000000001, "Latitude (1.0)");
        assertDoubleEquals(-1.0, sensorData.latLon.longitude, 0.000000001, "Longitude (-1.0)");
    }

    void testVelocityOnly() {
        std::cout << std::endl << "--- Testing Velocity Only ---" << std::endl;
        
        std::vector<uint8_t> payload;
        // VelocityXYZ (D012): Use simple values X=0.1, Y=0.2, Z=0.3
        payload.insert(payload.end(), {0xD0, 0x12, 0x12});
        
        // Generate correct FP16.32 values
        std::vector<uint8_t> vel01 = doubleToFP1632(0.1);
        std::vector<uint8_t> vel02 = doubleToFP1632(0.2);
        std::vector<uint8_t> vel03 = doubleToFP1632(0.3);
        
        payload.insert(payload.end(), vel01.begin(), vel01.end());
        payload.insert(payload.end(), vel02.begin(), vel02.end());
        payload.insert(payload.end(), vel03.begin(), vel03.end());
        
        std::vector<uint8_t> message = createMTData2Message(payload);
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message.data(), sensorData);
        
        assertTrue(success, "Velocity only parsing success");
        assertTrue(sensorData.hasVelocityXYZ, "Has VelocityXYZ");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertDoubleEquals(0.1, sensorData.velocityXYZ.velX, 0.000000001, "Velocity X (0.1)");
        assertDoubleEquals(0.2, sensorData.velocityXYZ.velY, 0.000000001, "Velocity Y (0.2)");
        assertDoubleEquals(0.3, sensorData.velocityXYZ.velZ, 0.000000001, "Velocity Z (0.3)");
    }

    void testUtcTimeOnly() {
        std::cout << std::endl << "--- Testing UTC Time Only ---" << std::endl;
        
        std::vector<uint8_t> payload;
        // UTC Time test data: 10 10 0C 2C A8 4D 3C 07 E9 07 0D 09 15 22 00
        payload.insert(payload.end(), {0x10, 0x10, 0x0C, 
                                      0x2C, 0xA8, 0x4D, 0x3C,  // nanoseconds: 749227324
                                      0x07, 0xE9,              // year: 2025
                                      0x07,                    // month: 7
                                      0x0D,                    // day: 13
                                      0x09,                    // hour: 9
                                      0x15,                    // minute: 21
                                      0x22,                    // second: 34
                                      0x00});                  // flags: 0
        
        std::vector<uint8_t> message = createMTData2Message(payload);
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message.data(), sensorData);
        
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
        std::cout << std::endl << "--- Testing Quaternion Only ---" << std::endl;
        
        std::vector<uint8_t> payload;
        // Quaternion test data: 20 10 10 3F 7F FE F3 BA 9C 8E C3 3A FD 24 45 3B AA 72 59
        payload.insert(payload.end(), {0x20, 0x10, 0x10,
                                      0x3F, 0x7F, 0xFE, 0xF3,  // q0: 0.9999840
                                      0xBA, 0x9C, 0x8E, 0xC3,  // q1: -0.0011944
                                      0x3A, 0xFD, 0x24, 0x45,  // q2: 0.0019313
                                      0x3B, 0xAA, 0x72, 0x59}); // q3: 0.0052016
        
        std::vector<uint8_t> message = createMTData2Message(payload);
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message.data(), sensorData);
        
        assertTrue(success, "Quaternion parsing success");
        assertTrue(sensorData.hasQuaternion, "Has Quaternion");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertFloatEquals(0.9999840f, sensorData.quaternion.q0, 0.0000001f, "Quaternion q0");
        assertFloatEquals(-0.0011944f, sensorData.quaternion.q1, 0.0000001f, "Quaternion q1");
        assertFloatEquals(0.0019313f, sensorData.quaternion.q2, 0.0000001f, "Quaternion q2");
        assertFloatEquals(0.0052016f, sensorData.quaternion.q3, 0.0000001f, "Quaternion q3");
    }

    void testBarometricPressureOnly() {
        std::cout << std::endl << "--- Testing Barometric Pressure Only ---" << std::endl;
        
        std::vector<uint8_t> payload;
        // Barometric pressure test data: 30 10 04 00 01 87 A4
        payload.insert(payload.end(), {0x30, 0x10, 0x04,
                                      0x00, 0x01, 0x87, 0xA4}); // pressure: 100260
        
        std::vector<uint8_t> message = createMTData2Message(payload);
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(message.data(), sensorData);
        
        assertTrue(success, "Barometric pressure parsing success");
        assertTrue(sensorData.hasBarometricPressure, "Has BarometricPressure");
        assertTrue(!sensorData.hasEulerAngles, "No EulerAngles");
        
        assertUint32Equals(100260, sensorData.barometricPressure.pressure, "Barometric pressure value");
    }

    void testInvalidMessage() {
        std::cout << std::endl << "--- Testing Invalid Message ---" << std::endl;
        
        // Test with invalid preamble
        std::vector<uint8_t> invalidMessage = {0xFF, 0xFF, 0x36, 0x00, 0x00};
        SensorData sensorData;
        bool success = XbusParser::parseMTData2(invalidMessage.data(), sensorData);
        assertTrue(!success, "Invalid preamble rejection");
        
        // Test with wrong message ID
        std::vector<uint8_t> wrongMsgId = {0xFA, 0xFF, 0x01, 0x00, 0x00};
        success = XbusParser::parseMTData2(wrongMsgId.data(), sensorData);
        assertTrue(!success, "Wrong message ID rejection");
    }
};

int main() {
    XbusParserTest test;
    test.runAllTests();
    return 0;
}