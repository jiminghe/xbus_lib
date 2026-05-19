#include "xbus.h"

bool xbus_check_preamble(const uint8_t* m) {
    return m[XBUS_OFFSET_TO_PREAMBLE] == XBUS_PREAMBLE;
}

int xbus_get_bus_id(const uint8_t* m) {
    return m[XBUS_OFFSET_TO_BID] & 0xff;
}

void xbus_set_bus_id(uint8_t* m, uint8_t v) {
    m[XBUS_OFFSET_TO_BID] = v & 0xff;
}

int xbus_get_message_id(const uint8_t* m) {
    return m[XBUS_OFFSET_TO_MID] & 0xff;
}

void xbus_set_message_id(uint8_t* m, uint8_t v) {
    m[XBUS_OFFSET_TO_MID] = v & 0xff;
}

int xbus_get_payload_length(const uint8_t* m) {
    int length = m[XBUS_OFFSET_TO_LEN] & 0xff;
    if (length != XBUS_LENGTH_EXTENDER_BYTE) {
        return length;
    }
    int result = m[XBUS_OFFSET_TO_LEN + 2] & 0xff;
    result += (m[XBUS_OFFSET_TO_LEN + 1] & 0xff) << 8;
    return result;
}

void xbus_set_payload_length(uint8_t* m, uint16_t len) {
    if (len < 255) {
        m[XBUS_OFFSET_TO_LEN] = (uint8_t)(len & 0xff);
    } else {
        m[XBUS_OFFSET_TO_LEN]     = XBUS_LENGTH_EXTENDER_BYTE;
        m[XBUS_OFFSET_TO_LEN + 1] = (uint8_t)((len >> 8) & 0xff);
        m[XBUS_OFFSET_TO_LEN + 2] = (uint8_t)(len & 0xff);
    }
}

void xbus_create_message(uint8_t* m, uint8_t bid, uint8_t mid, uint16_t len) {
    m[0] = XBUS_PREAMBLE;
    xbus_set_bus_id(m, bid);
    xbus_set_message_id(m, mid);
    xbus_set_payload_length(m, len);
}

int xbus_get_raw_length(const uint8_t* m) {
    int result = xbus_get_payload_length(m);
    if ((m[XBUS_OFFSET_TO_LEN] & 0xff) == XBUS_LENGTH_EXTENDER_BYTE) {
        result += 7;
    } else {
        result += 5;
    }
    return result;
}

uint8_t* xbus_get_pointer_to_payload(uint8_t* m) {
    if ((m[XBUS_OFFSET_TO_LEN] & 0xff) == XBUS_LENGTH_EXTENDER_BYTE) {
        return m + XBUS_OFFSET_TO_PAYLOAD_EXT;
    }
    return m + XBUS_OFFSET_TO_PAYLOAD;
}

const uint8_t* xbus_get_const_pointer_to_payload(const uint8_t* m) {
    return xbus_get_pointer_to_payload((uint8_t*)m);
}

void xbus_insert_checksum(uint8_t* m) {
    int n_bytes = xbus_get_raw_length(m);
    uint8_t checksum = 0;
    for (int i = 0; i < n_bytes - 2; i++) {
        checksum -= m[1 + i];
    }
    m[n_bytes - 1] = checksum;
}

bool xbus_verify_checksum(const uint8_t* m) {
    int n_bytes = xbus_get_raw_length(m);
    uint8_t checksum = 0;
    for (int n = 1; n < n_bytes; n++) {
        checksum += (uint8_t)(m[n] & 0xff);
    }
    return checksum == 0;
}
