#include "xsdevice_def.h"

#include "xbus/xbus.h"
#include "xbus/xbus_message_id.h"

#include <string.h>

static void write_u16_be(uint8_t* dst, uint16_t v) {
    dst[0] = (uint8_t)((v >> 8) & 0xFF);
    dst[1] = (uint8_t)( v       & 0xFF);
}

static void write_float_be(uint8_t* dst, float v) {
    uint32_t u;
    memcpy(&u, &v, 4);
    dst[0] = (uint8_t)((u >> 24) & 0xFF);
    dst[1] = (uint8_t)((u >> 16) & 0xFF);
    dst[2] = (uint8_t)((u >>  8) & 0xFF);
    dst[3] = (uint8_t)( u        & 0xFF);
}

#define TX_BUF_SIZE 256

static bool send_message(SerialReader* port, uint8_t mid,
                         const uint8_t* payload, uint16_t payload_len) {
    if (payload_len + 7u > TX_BUF_SIZE) return false;

    uint8_t buf[TX_BUF_SIZE];
    xbus_create_message(buf, XBUS_MASTERDEVICE, mid, payload_len);
    if (payload && payload_len > 0) {
        memcpy(xbus_get_pointer_to_payload(buf), payload, payload_len);
    }
    xbus_insert_checksum(buf);
    return serial_reader_write(port, buf, (size_t)xbus_get_raw_length(buf));
}

bool gotoConfig(SerialReader* port) {
    return send_message(port, XMID_GotoConfig, NULL, 0);
}

bool gotoMeasurement(SerialReader* port) {
    return send_message(port, XMID_GotoMeasurement, NULL, 0);
}

bool reqDid(SerialReader* port) {
    return send_message(port, XMID_ReqDid, NULL, 0);
}

bool reqFwVersion(SerialReader* port) {
    return send_message(port, XMID_ReqFirmwareRevision, NULL, 0);
}

bool setOutputConfiguration(SerialReader* port,
                            const XsOutputConfigItem* items, size_t count) {
    if (count > 0 && items == NULL) return false;
    if (count > (TX_BUF_SIZE - 7) / 4) return false;

    uint8_t payload[TX_BUF_SIZE];
    for (size_t i = 0; i < count; i++) {
        write_u16_be(&payload[i * 4 + 0], items[i].xdi);
        write_u16_be(&payload[i * 4 + 2], items[i].frequency);
    }
    return send_message(port, XMID_SetOutputConfig, payload, (uint16_t)(count * 4));
}

bool setAlignmentRotationQuaternion(SerialReader* port,
                                    SetRotationMatrix frame, const Quaternion* quat) {
    if (quat == NULL) return false;

    uint8_t payload[1 + 16];
    payload[0] = (uint8_t)frame;
    write_float_be(&payload[1],  quat->q0);
    write_float_be(&payload[5],  quat->q1);
    write_float_be(&payload[9],  quat->q2);
    write_float_be(&payload[13], quat->q3);

    return send_message(port, XMID_SetAlignmentRotation, payload, (uint16_t)sizeof(payload));
}
