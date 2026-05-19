#ifndef XBUS_PARSER_H
#define XBUS_PARSER_H

#include "xbus.h"
#include "xbus_message_id.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

typedef struct {
    double latitude;
    double longitude;
} LatLon;

typedef struct {
    double vel_x;
    double vel_y;
    double vel_z;
} VelocityXYZ;

typedef struct {
    float q0; /* w */
    float q1; /* x */
    float q2; /* y */
    float q3; /* z */
} Quaternion;

typedef struct {
    uint32_t nanoseconds;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  flags;
} UtcTime;

typedef struct {
    uint32_t pressure; /* Pa */
} BarometricPressure;

typedef struct {
    bool has_packet_counter;
    bool has_sample_time_fine;
    bool has_euler_angles;
    bool has_status_word;
    bool has_lat_lon;
    bool has_altitude_ellipsoid;
    bool has_velocity_xyz;
    bool has_utc_time;
    bool has_quaternion;
    bool has_barometric_pressure;

    uint16_t packet_counter;
    uint32_t sample_time_fine;
    EulerAngles euler_angles;
    uint32_t status_word;
    LatLon lat_lon;
    double altitude_ellipsoid;
    VelocityXYZ velocity_xyz;
    UtcTime utc_time;
    Quaternion quaternion;
    BarometricPressure barometric_pressure;
} SensorData;

/* XDI (Xsens Data Identifier) constants. */
#define XDI_PACKET_COUNTER      0x1020
#define XDI_SAMPLE_TIME_FINE    0x1060
#define XDI_EULER_ANGLES        0x2030
#define XDI_STATUS_WORD         0xE020
#define XDI_LAT_LON             0x5042
#define XDI_ALTITUDE_ELLIPSOID  0x5022
#define XDI_VELOCITY_XYZ        0xD012
#define XDI_QUATERNION          0x2010
#define XDI_ACCELERATION        0x4020
#define XDI_RATE_OF_TURN        0x8020
#define XDI_MAGNETIC_FIELD      0xC020
#define XDI_UTC_TIME            0x1010
#define XDI_BAROMETRIC_PRESSURE 0x3010

/* Big-endian readers. `index` is advanced by the number of bytes consumed. */
uint8_t  xbus_read_uint8 (const uint8_t* data, int* index);
uint16_t xbus_read_uint16(const uint8_t* data, int* index);
uint32_t xbus_read_uint32(const uint8_t* data, int* index);
float    xbus_read_float (const uint8_t* data, int* index);
double   xbus_read_fp1632(const uint8_t* data, int* index);

bool     xbus_parse_mt_data2          (const uint8_t* xbus_data, SensorData* out);
bool     xbus_parse_euler_angles      (const uint8_t* xbus_data, EulerAngles* out);
bool     xbus_parse_quaternion        (const uint8_t* xbus_data, Quaternion* out);
bool     xbus_parse_utc_time          (const uint8_t* xbus_data, UtcTime* out);
bool     xbus_parse_barometric_pressure(const uint8_t* xbus_data, BarometricPressure* out);
uint32_t xbus_parse_device_id         (const uint8_t* xbus_data);

/* snprintf-style formatters: write a NUL-terminated string into buf and return
   the would-be length (excluding the NUL), like vsnprintf. */
int xbus_format_message            (const uint8_t* xbus_data, char* buf, size_t buf_size);
int xbus_format_sensor_data        (const SensorData* data, char* buf, size_t buf_size);
int xbus_format_firmware_revision  (const uint8_t* xbus_data, char* buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* XBUS_PARSER_H */
