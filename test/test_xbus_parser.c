#include "xbus.h"
#include "xbus_message_id.h"
#include "xbus_parser.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_tests_passed = 0;
static int g_tests_total  = 0;

static void assert_true(int condition, const char* name) {
    g_tests_total++;
    if (condition) {
        g_tests_passed++;
        printf("[PASS] %s\n", name);
    } else {
        printf("[FAIL] %s\n", name);
    }
}

static void assert_double_equals(double expected, double actual, double tol, const char* name) {
    g_tests_total++;
    if (fabs(expected - actual) <= tol) {
        g_tests_passed++;
        printf("[PASS] %s (expected: %g, actual: %g)\n", name, expected, actual);
    } else {
        printf("[FAIL] %s (expected: %g, actual: %g, diff: %g)\n",
               name, expected, actual, fabs(expected - actual));
    }
}

static void assert_float_equals(float expected, float actual, float tol, const char* name) {
    g_tests_total++;
    if (fabsf(expected - actual) <= tol) {
        g_tests_passed++;
        printf("[PASS] %s (expected: %g, actual: %g)\n", name, (double)expected, (double)actual);
    } else {
        printf("[FAIL] %s (expected: %g, actual: %g, diff: %g)\n",
               name, (double)expected, (double)actual, (double)fabsf(expected - actual));
    }
}

static void assert_u32_equals(uint32_t expected, uint32_t actual, const char* name) {
    g_tests_total++;
    if (expected == actual) {
        g_tests_passed++;
        printf("[PASS] %s (expected: %u, actual: %u)\n", name, expected, actual);
    } else {
        printf("[FAIL] %s (expected: %u, actual: %u)\n", name, expected, actual);
    }
}

static void assert_u16_equals(uint16_t expected, uint16_t actual, const char* name) {
    g_tests_total++;
    if (expected == actual) {
        g_tests_passed++;
        printf("[PASS] %s (expected: %u, actual: %u)\n", name, expected, actual);
    } else {
        printf("[FAIL] %s (expected: %u, actual: %u)\n", name, expected, actual);
    }
}

static void assert_u8_equals(uint8_t expected, uint8_t actual, const char* name) {
    g_tests_total++;
    if (expected == actual) {
        g_tests_passed++;
        printf("[PASS] %s (expected: %u, actual: %u)\n", name, expected, actual);
    } else {
        printf("[FAIL] %s (expected: %u, actual: %u)\n", name, expected, actual);
    }
}

static void double_to_fp1632(double value, uint8_t out[6]) {
    int64_t fixed = (int64_t)llround(value * 4294967296.0);
    uint32_t fractional = (uint32_t)(fixed & 0xFFFFFFFFLL);
    int16_t  integer    = (int16_t)((fixed >> 32) & 0xFFFF);

    out[0] = (uint8_t)((fractional >> 24) & 0xFF);
    out[1] = (uint8_t)((fractional >> 16) & 0xFF);
    out[2] = (uint8_t)((fractional >> 8)  & 0xFF);
    out[3] = (uint8_t)( fractional        & 0xFF);
    out[4] = (uint8_t)(((uint16_t)integer >> 8) & 0xFF);
    out[5] = (uint8_t)( (uint16_t)integer       & 0xFF);
}

static size_t create_mt_data2_message(const uint8_t* payload, size_t payload_len, uint8_t* out) {
    size_t i = 0;
    out[i++] = XBUS_PREAMBLE;
    out[i++] = XBUS_MASTERDEVICE;
    out[i++] = XMID_MtData2;
    out[i++] = (uint8_t)payload_len;
    memcpy(out + i, payload, payload_len);
    i += payload_len;

    uint8_t checksum = 0;
    for (size_t j = 1; j < i; j++) checksum -= out[j];
    out[i++] = checksum;
    return i;
}

static size_t append_bytes(uint8_t* buf, size_t pos, const uint8_t* src, size_t n) {
    memcpy(buf + pos, src, n);
    return pos + n;
}

static void test_fp1632_conversion(void) {
    printf("\n--- Testing FP1632 Conversion ---\n");

    uint8_t lat_data[6] = {0x64, 0xA6, 0x8A, 0xA8, 0x00, 0x1F};
    int idx = 0;
    double lat = xbus_read_fp1632(lat_data, &idx);
    assert_double_equals(31.393166223541, lat, 1e-12, "Latitude FP1632 conversion");

    uint8_t lon_data[6] = {0x3A, 0xD0, 0x1E, 0xFC, 0x00, 0x79};
    idx = 0;
    double lon = xbus_read_fp1632(lon_data, &idx);
    assert_double_equals(121.229738174938, lon, 1e-12, "Longitude FP1632 conversion");

    uint8_t alt_data[6] = {0xB7, 0x0B, 0x3C, 0xEB, 0x00, 0x38};
    idx = 0;
    double alt = xbus_read_fp1632(alt_data, &idx);
    assert_double_equals(56.714969451306, alt, 1e-4, "Altitude FP1632 conversion");
}

static void test_mt_data2_all_components(void) {
    printf("\n--- Testing MTData2 with All Components ---\n");

    uint8_t payload[256];
    size_t p = 0;

    /* PacketCounter (1020): 2826 */
    {
        const uint8_t b[] = {0x10, 0x20, 0x02, 0x0B, 0x0A};
        p = append_bytes(payload, p, b, sizeof(b));
    }
    /* SampleTimeFine (1060): 12931224 */
    {
        const uint8_t b[] = {0x10, 0x60, 0x04, 0x00, 0xC5, 0x50, 0x98};
        p = append_bytes(payload, p, b, sizeof(b));
    }
    /* EulerAngles (2030) */
    {
        const uint8_t b[] = {0x20, 0x30, 0x0C,
                             0x43, 0x33, 0xEE, 0xEA,
                             0xBF, 0x93, 0x44, 0xFA,
                             0xC0, 0x15, 0xE3, 0x57};
        p = append_bytes(payload, p, b, sizeof(b));
    }
    /* StatusWord (E020): 2 */
    {
        const uint8_t b[] = {0xE0, 0x20, 0x04, 0x00, 0x00, 0x00, 0x02};
        p = append_bytes(payload, p, b, sizeof(b));
    }
    /* LatLon (5042) */
    {
        const uint8_t b[] = {0x50, 0x42, 0x0C,
                             0x64, 0xA6, 0x8A, 0xA8, 0x00, 0x1F,
                             0x3A, 0xD0, 0x1E, 0xFC, 0x00, 0x79};
        p = append_bytes(payload, p, b, sizeof(b));
    }
    /* AltitudeEllipsoid (5022) */
    {
        const uint8_t b[] = {0x50, 0x22, 0x06, 0xB7, 0x0B, 0x3C, 0xEB, 0x00, 0x38};
        p = append_bytes(payload, p, b, sizeof(b));
    }
    /* VelocityXYZ (D012) */
    {
        const uint8_t b[] = {0xD0, 0x12, 0x12,
                             0xFA, 0x7C, 0x28, 0x88, 0xFF, 0xFF,
                             0x03, 0x85, 0xF5, 0x88, 0x00, 0x00,
                             0xF4, 0xDD, 0xEB, 0x10, 0xFF, 0xFF};
        p = append_bytes(payload, p, b, sizeof(b));
    }

    uint8_t msg[512];
    create_mt_data2_message(payload, p, msg);

    SensorData sd;
    int ok = xbus_parse_mt_data2(msg, &sd);
    assert_true(ok, "MTData2 parsing success");
    assert_true(sd.has_packet_counter,     "Has PacketCounter");
    assert_true(sd.has_sample_time_fine,   "Has SampleTimeFine");
    assert_true(sd.has_euler_angles,       "Has EulerAngles");
    assert_true(sd.has_status_word,        "Has StatusWord");
    assert_true(sd.has_lat_lon,            "Has LatLon");
    assert_true(sd.has_altitude_ellipsoid, "Has AltitudeEllipsoid");
    assert_true(sd.has_velocity_xyz,       "Has VelocityXYZ");

    assert_true(sd.packet_counter   == 2826,     "PacketCounter value");
    assert_true(sd.sample_time_fine == 12931224, "SampleTimeFine value");
    assert_true(sd.status_word      == 2,        "StatusWord value");

    assert_float_equals(179.9332581f, sd.euler_angles.roll,  1e-4f, "Euler Roll");
    assert_float_equals(-1.1505425f,  sd.euler_angles.pitch, 1e-4f, "Euler Pitch");
    assert_float_equals(-2.3420007f,  sd.euler_angles.yaw,   1e-4f, "Euler Yaw");

    assert_double_equals(31.393166223541,  sd.lat_lon.latitude,    1e-12, "Latitude");
    assert_double_equals(121.229738174938, sd.lat_lon.longitude,   1e-12, "Longitude");
    assert_double_equals(56.714969451306,  sd.altitude_ellipsoid,  1e-4,  "Altitude");

    assert_double_equals(-0.021542994305, sd.velocity_xyz.vel_x, 1e-12, "Velocity X");
    assert_double_equals( 0.013762803748, sd.velocity_xyz.vel_y, 1e-12, "Velocity Y");
    assert_double_equals(-0.043488796800, sd.velocity_xyz.vel_z, 1e-12, "Velocity Z");
}

static void test_euler_only(void) {
    printf("\n--- Testing Euler Angles Only ---\n");

    const uint8_t payload[] = {
        0x20, 0x30, 0x0C,
        0x42, 0x34, 0x00, 0x00,
        0x41, 0xF0, 0x00, 0x00,
        0x42, 0xB4, 0x00, 0x00
    };
    uint8_t msg[64];
    create_mt_data2_message(payload, sizeof(payload), msg);

    SensorData sd;
    assert_true(xbus_parse_mt_data2(msg, &sd), "Euler only parsing success");
    assert_true(sd.has_euler_angles, "Has EulerAngles");
    assert_true(!sd.has_lat_lon,     "No LatLon");
    assert_true(!sd.has_velocity_xyz,"No VelocityXYZ");

    assert_float_equals(45.0f, sd.euler_angles.roll,  1e-3f, "Euler Roll (45.0)");
    assert_float_equals(30.0f, sd.euler_angles.pitch, 1e-3f, "Euler Pitch (30.0)");
    assert_float_equals(90.0f, sd.euler_angles.yaw,   1e-3f, "Euler Yaw (90.0)");
}

static void test_lat_lon_only(void) {
    printf("\n--- Testing LatLon Only ---\n");

    uint8_t payload[64];
    size_t p = 0;
    const uint8_t header[] = {0x50, 0x42, 0x0C};
    p = append_bytes(payload, p, header, sizeof(header));

    uint8_t lat_bytes[6], lon_bytes[6];
    double_to_fp1632( 1.0, lat_bytes);
    double_to_fp1632(-1.0, lon_bytes);
    p = append_bytes(payload, p, lat_bytes, 6);
    p = append_bytes(payload, p, lon_bytes, 6);

    uint8_t msg[64];
    create_mt_data2_message(payload, p, msg);

    SensorData sd;
    assert_true(xbus_parse_mt_data2(msg, &sd), "LatLon only parsing success");
    assert_true(sd.has_lat_lon,      "Has LatLon");
    assert_true(!sd.has_euler_angles,"No EulerAngles");
    assert_double_equals( 1.0, sd.lat_lon.latitude,  1e-9, "Latitude (1.0)");
    assert_double_equals(-1.0, sd.lat_lon.longitude, 1e-9, "Longitude (-1.0)");
}

static void test_velocity_only(void) {
    printf("\n--- Testing Velocity Only ---\n");

    uint8_t payload[64];
    size_t p = 0;
    const uint8_t header[] = {0xD0, 0x12, 0x12};
    p = append_bytes(payload, p, header, sizeof(header));

    uint8_t vx[6], vy[6], vz[6];
    double_to_fp1632(0.1, vx);
    double_to_fp1632(0.2, vy);
    double_to_fp1632(0.3, vz);
    p = append_bytes(payload, p, vx, 6);
    p = append_bytes(payload, p, vy, 6);
    p = append_bytes(payload, p, vz, 6);

    uint8_t msg[64];
    create_mt_data2_message(payload, p, msg);

    SensorData sd;
    assert_true(xbus_parse_mt_data2(msg, &sd), "Velocity only parsing success");
    assert_true(sd.has_velocity_xyz, "Has VelocityXYZ");
    assert_true(!sd.has_euler_angles,"No EulerAngles");
    assert_double_equals(0.1, sd.velocity_xyz.vel_x, 1e-9, "Velocity X (0.1)");
    assert_double_equals(0.2, sd.velocity_xyz.vel_y, 1e-9, "Velocity Y (0.2)");
    assert_double_equals(0.3, sd.velocity_xyz.vel_z, 1e-9, "Velocity Z (0.3)");
}

static void test_utc_only(void) {
    printf("\n--- Testing UTC Time Only ---\n");

    const uint8_t payload[] = {
        0x10, 0x10, 0x0C,
        0x2C, 0xA8, 0x4D, 0x3C,
        0x07, 0xE9,
        0x07, 0x0D, 0x09, 0x15, 0x22, 0x00
    };
    uint8_t msg[64];
    create_mt_data2_message(payload, sizeof(payload), msg);

    SensorData sd;
    assert_true(xbus_parse_mt_data2(msg, &sd), "UTC Time parsing success");
    assert_true(sd.has_utc_time,      "Has UtcTime");
    assert_true(!sd.has_euler_angles, "No EulerAngles");

    assert_u32_equals(749227324, sd.utc_time.nanoseconds, "UTC nanoseconds");
    assert_u16_equals(2025,      sd.utc_time.year,        "UTC year");
    assert_u8_equals(7,          sd.utc_time.month,       "UTC month");
    assert_u8_equals(13,         sd.utc_time.day,         "UTC day");
    assert_u8_equals(9,          sd.utc_time.hour,        "UTC hour");
    assert_u8_equals(21,         sd.utc_time.minute,      "UTC minute");
    assert_u8_equals(34,         sd.utc_time.second,      "UTC second");
    assert_u8_equals(0,          sd.utc_time.flags,       "UTC flags");
}

static void test_quaternion_only(void) {
    printf("\n--- Testing Quaternion Only ---\n");

    const uint8_t payload[] = {
        0x20, 0x10, 0x10,
        0x3F, 0x7F, 0xFE, 0xF3,
        0xBA, 0x9C, 0x8E, 0xC3,
        0x3A, 0xFD, 0x24, 0x45,
        0x3B, 0xAA, 0x72, 0x59
    };
    uint8_t msg[64];
    create_mt_data2_message(payload, sizeof(payload), msg);

    SensorData sd;
    assert_true(xbus_parse_mt_data2(msg, &sd), "Quaternion parsing success");
    assert_true(sd.has_quaternion,    "Has Quaternion");
    assert_true(!sd.has_euler_angles, "No EulerAngles");

    assert_float_equals(0.9999840f,  sd.quaternion.q0, 1e-7f, "Quaternion q0");
    assert_float_equals(-0.0011944f, sd.quaternion.q1, 1e-7f, "Quaternion q1");
    assert_float_equals(0.0019313f,  sd.quaternion.q2, 1e-7f, "Quaternion q2");
    assert_float_equals(0.0052016f,  sd.quaternion.q3, 1e-7f, "Quaternion q3");
}

static void test_baro_only(void) {
    printf("\n--- Testing Barometric Pressure Only ---\n");

    const uint8_t payload[] = {
        0x30, 0x10, 0x04,
        0x00, 0x01, 0x87, 0xA4
    };
    uint8_t msg[64];
    create_mt_data2_message(payload, sizeof(payload), msg);

    SensorData sd;
    assert_true(xbus_parse_mt_data2(msg, &sd), "Barometric pressure parsing success");
    assert_true(sd.has_barometric_pressure, "Has BarometricPressure");
    assert_true(!sd.has_euler_angles,       "No EulerAngles");

    assert_u32_equals(100260, sd.barometric_pressure.pressure, "Barometric pressure value");
}

static void test_invalid_message(void) {
    printf("\n--- Testing Invalid Message ---\n");

    const uint8_t bad_preamble[] = {0xFF, 0xFF, 0x36, 0x00, 0x00};
    SensorData sd;
    assert_true(!xbus_parse_mt_data2(bad_preamble, &sd), "Invalid preamble rejection");

    const uint8_t wrong_mid[] = {0xFA, 0xFF, 0x01, 0x00, 0x00};
    assert_true(!xbus_parse_mt_data2(wrong_mid, &sd), "Wrong message ID rejection");
}

int main(void) {
    printf("=== XBus Parser Test Suite ===\n");

    test_mt_data2_all_components();
    test_fp1632_conversion();
    test_euler_only();
    test_lat_lon_only();
    test_velocity_only();
    test_utc_only();
    test_quaternion_only();
    test_baro_only();
    test_invalid_message();

    printf("\n=== Test Results ===\n");
    printf("Passed: %d/%d\n", g_tests_passed, g_tests_total);
    if (g_tests_passed == g_tests_total) {
        printf("All tests PASSED!\n");
        return 0;
    }
    printf("Some tests FAILED!\n");
    return 1;
}
