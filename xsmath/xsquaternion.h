#ifndef XSQUATERNION_H
#define XSQUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float w;
    float x;
    float y;
    float z;
} XsQuaternion;

/* dest = left * right (Hamilton product). Aliasing dest with left or right is OK. */
void XsQuaternion_multiply(const XsQuaternion* left,
                           const XsQuaternion* right,
                           XsQuaternion* dest);

#ifdef __cplusplus
}
#endif

#endif /* XSQUATERNION_H */
