#ifndef XBUS_H
#define XBUS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XBUS_OFFSET_TO_PREAMBLE    0
#define XBUS_OFFSET_TO_BID         1
#define XBUS_OFFSET_TO_MID         2
#define XBUS_OFFSET_TO_LEN         3
#define XBUS_OFFSET_TO_LEN_EXT_HI  4
#define XBUS_OFFSET_TO_LEN_EXT_LO  5
#define XBUS_OFFSET_TO_PAYLOAD     4
#define XBUS_OFFSET_TO_PAYLOAD_EXT 6
#define XBUS_CHECKSUM_SIZE         1
#define XBUS_LENGTH_EXTENDER_BYTE  0xFF
#define XBUS_PREAMBLE              0xFA
#define XBUS_MASTERDEVICE          0xFF
#define XBUS_EXTENDED_LENGTH       0xFF

bool xbus_check_preamble(const uint8_t* message);

int  xbus_get_bus_id(const uint8_t* message);
void xbus_set_bus_id(uint8_t* message, uint8_t bus_id);

int  xbus_get_message_id(const uint8_t* message);
void xbus_set_message_id(uint8_t* message, uint8_t message_id);

int  xbus_get_payload_length(const uint8_t* message);
void xbus_set_payload_length(uint8_t* message, uint16_t payload_length);

void xbus_create_message(uint8_t* message, uint8_t bid, uint8_t mid, uint16_t len);

int xbus_get_raw_length(const uint8_t* message);

uint8_t*       xbus_get_pointer_to_payload(uint8_t* message);
const uint8_t* xbus_get_const_pointer_to_payload(const uint8_t* message);

void xbus_insert_checksum(uint8_t* message);
bool xbus_verify_checksum(const uint8_t* message);

#ifdef __cplusplus
}
#endif

#endif /* XBUS_H */
