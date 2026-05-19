#include "xbus_parser.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

uint8_t xbus_read_uint8(const uint8_t* data, int* index) {
    return data[(*index)++];
}

uint16_t xbus_read_uint16(const uint8_t* data, int* index) {
    uint16_t r = 0;
    r |= ((uint16_t)data[(*index)++]) << 8;
    r |= ((uint16_t)data[(*index)++]) << 0;
    return r;
}

uint32_t xbus_read_uint32(const uint8_t* data, int* index) {
    uint32_t r = 0;
    r |= ((uint32_t)data[(*index)++]) << 24;
    r |= ((uint32_t)data[(*index)++]) << 16;
    r |= ((uint32_t)data[(*index)++]) << 8;
    r |= ((uint32_t)data[(*index)++]) << 0;
    return r;
}

float xbus_read_float(const uint8_t* data, int* index) {
    uint32_t tmp = xbus_read_uint32(data, index);
    float r;
    memcpy(&r, &tmp, 4);
    return r;
}

double xbus_read_fp1632(const uint8_t* data, int* index) {
    /* FP16.32: 4-byte fractional (big-endian) + 2-byte signed integer (big-endian);
       value = ((int)integer << 32 | fractional) / 2^32. */
    uint32_t fractional = xbus_read_uint32(data, index);
    int16_t integer = (int16_t)xbus_read_uint16(data, index);
    int64_t fixed = ((int64_t)integer << 32) | ((int64_t)fractional & 0xFFFFFFFFLL);
    return (double)fixed / 4294967296.0;
}

bool xbus_parse_mt_data2(const uint8_t* xbus_data, SensorData* sd) {
    if (!xbus_check_preamble(xbus_data)) return false;
    if (xbus_get_message_id(xbus_data) != XMID_MtData2) return false;

    memset(sd, 0, sizeof(*sd));

    int payload_length = xbus_get_payload_length(xbus_data);
    const uint8_t* payload = xbus_get_const_pointer_to_payload(xbus_data);

    int index = 0;
    while (index < payload_length) {
        if (index + 3 > payload_length) break;
        uint16_t xdi = xbus_read_uint16(payload, &index);
        uint8_t size = xbus_read_uint8(payload, &index);
        if (index + size > payload_length) break;

        switch (xdi) {
        case XDI_PACKET_COUNTER:
            if (size == 2) {
                sd->packet_counter = xbus_read_uint16(payload, &index);
                sd->has_packet_counter = true;
            } else index += size;
            break;
        case XDI_SAMPLE_TIME_FINE:
            if (size == 4) {
                sd->sample_time_fine = xbus_read_uint32(payload, &index);
                sd->has_sample_time_fine = true;
            } else index += size;
            break;
        case XDI_EULER_ANGLES:
            if (size == 12) {
                sd->euler_angles.roll  = xbus_read_float(payload, &index);
                sd->euler_angles.pitch = xbus_read_float(payload, &index);
                sd->euler_angles.yaw   = xbus_read_float(payload, &index);
                sd->has_euler_angles = true;
            } else index += size;
            break;
        case XDI_STATUS_WORD:
            if (size == 4) {
                sd->status_word = xbus_read_uint32(payload, &index);
                sd->has_status_word = true;
            } else index += size;
            break;
        case XDI_LAT_LON:
            if (size == 12) {
                sd->lat_lon.latitude  = xbus_read_fp1632(payload, &index);
                sd->lat_lon.longitude = xbus_read_fp1632(payload, &index);
                sd->has_lat_lon = true;
            } else index += size;
            break;
        case XDI_ALTITUDE_ELLIPSOID:
            if (size == 6) {
                sd->altitude_ellipsoid = xbus_read_fp1632(payload, &index);
                sd->has_altitude_ellipsoid = true;
            } else index += size;
            break;
        case XDI_VELOCITY_XYZ:
            if (size == 18) {
                sd->velocity_xyz.vel_x = xbus_read_fp1632(payload, &index);
                sd->velocity_xyz.vel_y = xbus_read_fp1632(payload, &index);
                sd->velocity_xyz.vel_z = xbus_read_fp1632(payload, &index);
                sd->has_velocity_xyz = true;
            } else index += size;
            break;
        case XDI_UTC_TIME:
            if (size == 12) {
                sd->utc_time.nanoseconds = xbus_read_uint32(payload, &index);
                sd->utc_time.year   = xbus_read_uint16(payload, &index);
                sd->utc_time.month  = xbus_read_uint8(payload, &index);
                sd->utc_time.day    = xbus_read_uint8(payload, &index);
                sd->utc_time.hour   = xbus_read_uint8(payload, &index);
                sd->utc_time.minute = xbus_read_uint8(payload, &index);
                sd->utc_time.second = xbus_read_uint8(payload, &index);
                sd->utc_time.flags  = xbus_read_uint8(payload, &index);
                sd->has_utc_time = true;
            } else index += size;
            break;
        case XDI_QUATERNION:
            if (size == 16) {
                sd->quaternion.q0 = xbus_read_float(payload, &index);
                sd->quaternion.q1 = xbus_read_float(payload, &index);
                sd->quaternion.q2 = xbus_read_float(payload, &index);
                sd->quaternion.q3 = xbus_read_float(payload, &index);
                sd->has_quaternion = true;
            } else index += size;
            break;
        case XDI_BAROMETRIC_PRESSURE:
            if (size == 4) {
                sd->barometric_pressure.pressure = xbus_read_uint32(payload, &index);
                sd->has_barometric_pressure = true;
            } else index += size;
            break;
        default:
            index += size;
            break;
        }
    }
    return true;
}

bool xbus_parse_euler_angles(const uint8_t* xbus_data, EulerAngles* out) {
    SensorData sd;
    if (xbus_parse_mt_data2(xbus_data, &sd) && sd.has_euler_angles) {
        *out = sd.euler_angles;
        return true;
    }
    return false;
}

bool xbus_parse_quaternion(const uint8_t* xbus_data, Quaternion* out) {
    SensorData sd;
    if (xbus_parse_mt_data2(xbus_data, &sd) && sd.has_quaternion) {
        *out = sd.quaternion;
        return true;
    }
    return false;
}

bool xbus_parse_utc_time(const uint8_t* xbus_data, UtcTime* out) {
    SensorData sd;
    if (xbus_parse_mt_data2(xbus_data, &sd) && sd.has_utc_time) {
        *out = sd.utc_time;
        return true;
    }
    return false;
}

bool xbus_parse_barometric_pressure(const uint8_t* xbus_data, BarometricPressure* out) {
    SensorData sd;
    if (xbus_parse_mt_data2(xbus_data, &sd) && sd.has_barometric_pressure) {
        *out = sd.barometric_pressure;
        return true;
    }
    return false;
}

uint32_t xbus_parse_device_id(const uint8_t* xbus_data) {
    if (!xbus_check_preamble(xbus_data)) return 0;
    if (xbus_get_message_id(xbus_data) != XMID_DeviceId) return 0;
    int index = 4;
    return xbus_read_uint32(xbus_data, &index);
}

/* snprintf-chain helper: appends at offset w, returns updated total. Past the
   end it stops writing but still tracks the would-be length. */
static int sappend(char* buf, size_t bs, int w, const char* fmt, ...) {
    if (w < 0) return w;
    va_list ap;
    if (buf == NULL || bs == 0 || (size_t)w >= bs) {
        va_start(ap, fmt);
        int n = vsnprintf(NULL, 0, fmt, ap);
        va_end(ap);
        return w + (n > 0 ? n : 0);
    }
    va_start(ap, fmt);
    int n = vsnprintf(buf + w, bs - (size_t)w, fmt, ap);
    va_end(ap);
    if (n < 0) return w;
    return w + n;
}

static int format_status_word(char* buf, size_t bs, int w, uint32_t sw) {
    w = sappend(buf, bs, w, "0x%08X", (unsigned)sw);
    if (sw & 0x0001u) w = sappend(buf, bs, w, " [SelfTest]");
    if (sw & 0x0002u) w = sappend(buf, bs, w, " [FilterValid]");
    if (sw & 0x0004u) w = sappend(buf, bs, w, " [GNSSFix]");
    return w;
}

static int format_utc_time(char* buf, size_t bs, int w, const UtcTime* t) {
    w = sappend(buf, bs, w, "%04u-%02u-%02u %02u:%02u:%02u.%09u",
                (unsigned)t->year, (unsigned)t->month, (unsigned)t->day,
                (unsigned)t->hour, (unsigned)t->minute, (unsigned)t->second,
                (unsigned)t->nanoseconds);
    if (t->flags) w = sappend(buf, bs, w, " [F:%02X]", (unsigned)t->flags);
    return w;
}

static int format_quaternion(char* buf, size_t bs, int w, const Quaternion* q) {
    return sappend(buf, bs, w, "(%.6f, %.6f, %.6f, %.6f)",
                   (double)q->q0, (double)q->q1, (double)q->q2, (double)q->q3);
}

static int format_baro(char* buf, size_t bs, int w, const BarometricPressure* b) {
    return sappend(buf, bs, w, "%.2f hPa", b->pressure / 100.0);
}

static int format_sensor_data_at(char* buf, size_t bs, int w, const SensorData* d) {
    bool first = true;

    if (d->has_packet_counter) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "PC=%u", (unsigned)d->packet_counter);
    }
    if (d->has_sample_time_fine) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "STF=%u", (unsigned)d->sample_time_fine);
    }
    if (d->has_utc_time) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "UTC=");
        w = format_utc_time(buf, bs, w, &d->utc_time);
    }
    if (d->has_euler_angles) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "Euler(R=%.2f\xc2\xb0, P=%.2f\xc2\xb0, Y=%.2f\xc2\xb0)",
                    (double)d->euler_angles.roll,
                    (double)d->euler_angles.pitch,
                    (double)d->euler_angles.yaw);
    }
    if (d->has_quaternion) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "Quat=");
        w = format_quaternion(buf, bs, w, &d->quaternion);
    }
    if (d->has_lat_lon) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "LatLon(%.8f, %.8f)",
                    d->lat_lon.latitude, d->lat_lon.longitude);
    }
    if (d->has_altitude_ellipsoid) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "Alt=%.3fm", d->altitude_ellipsoid);
    }
    if (d->has_velocity_xyz) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "Vel(%.4f, %.4f, %.4f)m/s",
                    d->velocity_xyz.vel_x, d->velocity_xyz.vel_y, d->velocity_xyz.vel_z);
    }
    if (d->has_barometric_pressure) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "Baro=");
        w = format_baro(buf, bs, w, &d->barometric_pressure);
    }
    if (d->has_status_word) {
        if (!first) w = sappend(buf, bs, w, ", "); first = false;
        w = sappend(buf, bs, w, "Status=");
        w = format_status_word(buf, bs, w, d->status_word);
    }
    (void)first;
    return w;
}

int xbus_format_sensor_data(const SensorData* data, char* buf, size_t buf_size) {
    return format_sensor_data_at(buf, buf_size, 0, data);
}

int xbus_format_message(const uint8_t* xbus_data, char* buf, size_t bs) {
    if (!xbus_check_preamble(xbus_data)) {
        return sappend(buf, bs, 0, "Invalid xbus message");
    }
    int mid = xbus_get_message_id(xbus_data);
    int index = 4;

    switch (mid) {
    case XMID_Wakeup:              return sappend(buf, bs, 0, "XMID_Wakeup");
    case XMID_GotoConfigAck:       return sappend(buf, bs, 0, "XMID_GotoConfigAck");
    case XMID_GotoMeasurementAck:  return sappend(buf, bs, 0, "XMID_GotoMeasurementAck");
    case XMID_GotoBootLoaderAck:   return sappend(buf, bs, 0, "XMID_GotoBootLoaderAck");
    case XMID_FirmwareUpdate:      return sappend(buf, bs, 0, "XMID_FirmwareUpdate");
    case XMID_ResetAck:            return sappend(buf, bs, 0, "XMID_ResetAck");

    case XMID_DeviceId: {
        uint32_t did = xbus_read_uint32(xbus_data, &index);
        return sappend(buf, bs, 0, "XMID_DeviceId: 0x%08X", (unsigned)did);
    }
    case XMID_MtData2: {
        SensorData sd;
        if (xbus_parse_mt_data2(xbus_data, &sd)) {
            int w = sappend(buf, bs, 0, "XMID_MtData2: ");
            return format_sensor_data_at(buf, bs, w, &sd);
        }
        return sappend(buf, bs, 0, "XMID_MtData2: Failed to parse");
    }
    case XMID_FirmwareRevision: {
        uint8_t major = xbus_read_uint8(xbus_data, &index);
        uint8_t minor = xbus_read_uint8(xbus_data, &index);
        uint8_t patch = xbus_read_uint8(xbus_data, &index);
        return sappend(buf, bs, 0, "Firmware revision: %d.%d.%d",
                       (int)major, (int)minor, (int)patch);
    }
    default:
        return sappend(buf, bs, 0, "Unhandled xbus message: MessageId = 0x%02X",
                       (unsigned)mid);
    }
}

int xbus_format_firmware_revision(const uint8_t* xbus_data, char* buf, size_t bs) {
    if (!xbus_check_preamble(xbus_data) ||
        xbus_get_message_id(xbus_data) != XMID_FirmwareRevision) {
        return sappend(buf, bs, 0, "");
    }
    int index = 4;
    uint8_t major = xbus_read_uint8(xbus_data, &index);
    uint8_t minor = xbus_read_uint8(xbus_data, &index);
    uint8_t patch = xbus_read_uint8(xbus_data, &index);
    return sappend(buf, bs, 0, "%d.%d.%d", (int)major, (int)minor, (int)patch);
}
