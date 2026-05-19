#include "xseuler.h"

#include <math.h>

#define XS_PI       3.14159265358979323846f
#define XS_RAD2DEG  (180.0f / XS_PI)

static float clamped_asinf(float v) {
    if (v >=  1.0f) return  XS_PI * 0.5f;
    if (v <= -1.0f) return -XS_PI * 0.5f;
    return asinf(v);
}

void XsEuler_fromQuaternion(XsEuler* dest, const XsQuaternion* q) {
    float w = q->w, x = q->x, y = q->y, z = q->z;
    float sqw  = w * w;
    float dphi = 2.0f * (sqw + z * z) - 1.0f;
    float dpsi = 2.0f * (sqw + x * x) - 1.0f;

    dest->roll  =  XS_RAD2DEG * atan2f(2.0f * (y * z + w * x), dphi);
    dest->pitch = -XS_RAD2DEG * clamped_asinf(2.0f * (x * z - w * y));
    dest->yaw   =  XS_RAD2DEG * atan2f(2.0f * (x * y + w * z), dpsi);
}
