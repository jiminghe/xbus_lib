#include "xsquaternion.h"

void XsQuaternion_multiply(const XsQuaternion* left,
                           const XsQuaternion* right,
                           XsQuaternion* dest)
{
    float lw = left->w,  lx = left->x,  ly = left->y,  lz = left->z;
    float rw = right->w, rx = right->x, ry = right->y, rz = right->z;

    dest->w = lw * rw - lx * rx - ly * ry - lz * rz;
    dest->x = lx * rw + lw * rx - lz * ry + ly * rz;
    dest->y = ly * rw + lz * rx + lw * ry - lx * rz;
    dest->z = lz * rw - ly * rx + lx * ry + lw * rz;
}
