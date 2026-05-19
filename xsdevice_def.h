#ifndef XSDEVICE_DEF_H
#define XSDEVICE_DEF_H

#include "serial_reader.h"
#include "xbus/xbus_parser.h"  /* for Quaternion */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Alignment frame selector for SetAlignmentRotation. */
typedef enum {
    SRM_ROTSENSOR = 0x0,
    SRM_ROTLOCAL  = 0x1
} SetRotationMatrix;

/* One entry of a SetOutputConfig payload: data identifier + output rate (Hz). */
typedef struct {
    uint16_t xdi;
    uint16_t frequency;
} XsOutputConfigItem;

/* Each function builds the corresponding Xbus message and writes it on the
   serial port. Fire-and-forget: the caller drives the response/state machine
   itself (matching the embedded SDK pattern). Returns the result of the
   underlying serial write. */
bool gotoConfig                    (SerialReader* port);
bool gotoMeasurement               (SerialReader* port);
bool reqDid                        (SerialReader* port);
bool reqFwVersion                  (SerialReader* port);
bool setOutputConfiguration        (SerialReader* port,
                                    const XsOutputConfigItem* items, size_t count);
bool setAlignmentRotationQuaternion(SerialReader* port,
                                    SetRotationMatrix frame, const Quaternion* quat);

#ifdef __cplusplus
}
#endif

#endif /* XSDEVICE_DEF_H */
