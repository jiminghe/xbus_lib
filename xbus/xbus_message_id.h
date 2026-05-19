#ifndef XBUS_MESSAGE_ID_H
#define XBUS_MESSAGE_ID_H

#include <stdint.h>

/* Xsens Xbus message identifiers. */
#define XMID_Wakeup              0x3E
#define XMID_WakeupAck           0x3F
#define XMID_ReqDid              0x00
#define XMID_DeviceId            0x01
#define XMID_GotoConfig          0x30
#define XMID_GotoConfigAck       0x31
#define XMID_GotoMeasurement     0x10
#define XMID_GotoMeasurementAck  0x11
#define XMID_MtData2             0x36
#define XMID_ReqOutputConfig     0xC0
#define XMID_SetOutputConfig     0xC0
#define XMID_OutputConfig        0xC1
#define XMID_Reset               0x40
#define XMID_ResetAck            0x41
#define XMID_Error               0x42
#define XMID_ToggleIoPins        0xBE
#define XMID_ToggleIoPinsAck     0xBF
#define XMID_FirmwareUpdate      0xF2
#define XMID_GotoBootLoader      0xF0
#define XMID_GotoBootLoaderAck   0xF1
#define XMID_ReqFirmwareRevision 0x12
#define XMID_FirmwareRevision    0x13
#define XMID_SetAlignmentRotation       0xEC
#define XMID_SetAlignmentRotationAck    0xED

#endif /* XBUS_MESSAGE_ID_H */
