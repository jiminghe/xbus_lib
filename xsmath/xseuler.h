#ifndef XSEULER_H
#define XSEULER_H

#include "xsquaternion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll;   /* x, degrees */
    float pitch;  /* y, degrees */
    float yaw;    /* z, degrees */
} XsEuler;

/* Convert a quaternion to Euler angles in degrees (roll-pitch-yaw / X-Y-Z). */
void XsEuler_fromQuaternion(XsEuler* dest, const XsQuaternion* q);

#ifdef __cplusplus
}
#endif

#endif /* XSEULER_H */
