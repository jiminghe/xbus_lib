#include "xsquaternion.h"
#include "xseuler.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_tests_passed = 0;
static int g_tests_total  = 0;

static void assert_true(int condition, const char* name) {
    g_tests_total++;
    if (condition) {
        g_tests_passed++;
        printf("[PASS] %s\n", name);
    } else {
        printf("[FAIL] %s\n", name);
    }
}

static void assert_float_close(float expected, float actual, float tol, const char* name) {
    g_tests_total++;
    if (fabsf(expected - actual) <= tol) {
        g_tests_passed++;
        printf("[PASS] %s (expected: %g, actual: %g)\n", name,
               (double)expected, (double)actual);
    } else {
        printf("[FAIL] %s (expected: %g, actual: %g, diff: %g)\n", name,
               (double)expected, (double)actual,
               (double)fabsf(expected - actual));
    }
}

static void assert_quat_close(const XsQuaternion* expected,
                              const XsQuaternion* actual,
                              float tol, const char* name) {
    char label[128];
    snprintf(label, sizeof(label), "%s.w", name); assert_float_close(expected->w, actual->w, tol, label);
    snprintf(label, sizeof(label), "%s.x", name); assert_float_close(expected->x, actual->x, tol, label);
    snprintf(label, sizeof(label), "%s.y", name); assert_float_close(expected->y, actual->y, tol, label);
    snprintf(label, sizeof(label), "%s.z", name); assert_float_close(expected->z, actual->z, tol, label);
}

static void assert_euler_close(float roll, float pitch, float yaw,
                               const XsEuler* actual,
                               float tol, const char* name) {
    char label[128];
    snprintf(label, sizeof(label), "%s.roll",  name); assert_float_close(roll,  actual->roll,  tol, label);
    snprintf(label, sizeof(label), "%s.pitch", name); assert_float_close(pitch, actual->pitch, tol, label);
    snprintf(label, sizeof(label), "%s.yaw",   name); assert_float_close(yaw,   actual->yaw,   tol, label);
}

/* ---- XsQuaternion_multiply ---- */

static void test_multiply_identity_left(void) {
    printf("\n--- XsQuaternion_multiply: identity * q == q ---\n");
    XsQuaternion id = { 1.0f, 0.0f, 0.0f, 0.0f };
    XsQuaternion q  = { 0.5f, 0.3f, -0.4f, 0.7f };
    XsQuaternion out;
    XsQuaternion_multiply(&id, &q, &out);
    assert_quat_close(&q, &out, 1e-6f, "id*q");
}

static void test_multiply_identity_right(void) {
    printf("\n--- XsQuaternion_multiply: q * identity == q ---\n");
    XsQuaternion id = { 1.0f, 0.0f, 0.0f, 0.0f };
    XsQuaternion q  = { 0.5f, 0.3f, -0.4f, 0.7f };
    XsQuaternion out;
    XsQuaternion_multiply(&q, &id, &out);
    assert_quat_close(&q, &out, 1e-6f, "q*id");
}

static void test_multiply_basis_units(void) {
    printf("\n--- XsQuaternion_multiply: Hamilton basis i*j=k, j*k=i, k*i=j ---\n");
    XsQuaternion i = { 0.0f, 1.0f, 0.0f, 0.0f };
    XsQuaternion j = { 0.0f, 0.0f, 1.0f, 0.0f };
    XsQuaternion k = { 0.0f, 0.0f, 0.0f, 1.0f };
    XsQuaternion out;

    XsQuaternion_multiply(&i, &j, &out); assert_quat_close(&k, &out, 1e-6f, "i*j");
    XsQuaternion_multiply(&j, &k, &out); assert_quat_close(&i, &out, 1e-6f, "j*k");
    XsQuaternion_multiply(&k, &i, &out); assert_quat_close(&j, &out, 1e-6f, "k*i");
}

static void test_multiply_yaw90_squared(void) {
    printf("\n--- XsQuaternion_multiply: yaw90 * yaw90 == yaw180 ---\n");
    float c = cosf((float)M_PI / 4.0f);
    float s = sinf((float)M_PI / 4.0f);
    XsQuaternion yaw90  = { c, 0.0f, 0.0f, s };
    XsQuaternion yaw180 = { 0.0f, 0.0f, 0.0f, 1.0f };
    XsQuaternion out;
    XsQuaternion_multiply(&yaw90, &yaw90, &out);
    assert_quat_close(&yaw180, &out, 1e-6f, "yaw90^2");
}

static void test_multiply_aliasing(void) {
    printf("\n--- XsQuaternion_multiply: dest aliases left and right ---\n");
    float c = cosf((float)M_PI / 4.0f);
    float s = sinf((float)M_PI / 4.0f);
    XsQuaternion yaw180 = { 0.0f, 0.0f, 0.0f, 1.0f };

    /* dest == left */
    XsQuaternion a = { c, 0.0f, 0.0f, s };
    XsQuaternion b = { c, 0.0f, 0.0f, s };
    XsQuaternion_multiply(&a, &b, &a);
    assert_quat_close(&yaw180, &a, 1e-6f, "alias-left");

    /* dest == right */
    XsQuaternion a2 = { c, 0.0f, 0.0f, s };
    XsQuaternion b2 = { c, 0.0f, 0.0f, s };
    XsQuaternion_multiply(&a2, &b2, &b2);
    assert_quat_close(&yaw180, &b2, 1e-6f, "alias-right");

    /* dest == left == right (square self) */
    XsQuaternion sq = { c, 0.0f, 0.0f, s };
    XsQuaternion_multiply(&sq, &sq, &sq);
    assert_quat_close(&yaw180, &sq, 1e-6f, "alias-self");
}

/* ---- XsEuler_fromQuaternion ---- */

static void test_euler_identity(void) {
    printf("\n--- XsEuler_fromQuaternion: identity -> (0,0,0) ---\n");
    XsQuaternion id = { 1.0f, 0.0f, 0.0f, 0.0f };
    XsEuler e;
    XsEuler_fromQuaternion(&e, &id);
    assert_euler_close(0.0f, 0.0f, 0.0f, &e, 1e-4f, "identity");
}

static void test_euler_pure_roll_90(void) {
    printf("\n--- XsEuler_fromQuaternion: pure 90deg roll -> (90,0,0) ---\n");
    float c = cosf((float)M_PI / 4.0f);
    float s = sinf((float)M_PI / 4.0f);
    XsQuaternion q = { c, s, 0.0f, 0.0f };
    XsEuler e;
    XsEuler_fromQuaternion(&e, &q);
    assert_euler_close(90.0f, 0.0f, 0.0f, &e, 1e-3f, "roll90");
}

static void test_euler_pure_pitch_30(void) {
    printf("\n--- XsEuler_fromQuaternion: pure 30deg pitch -> (0,30,0) ---\n");
    float c = cosf((float)(M_PI / 12.0));  /* cos(15deg) */
    float s = sinf((float)(M_PI / 12.0));  /* sin(15deg) */
    XsQuaternion q = { c, 0.0f, s, 0.0f };
    XsEuler e;
    XsEuler_fromQuaternion(&e, &q);
    assert_euler_close(0.0f, 30.0f, 0.0f, &e, 1e-3f, "pitch30");
}

static void test_euler_pure_yaw_90(void) {
    printf("\n--- XsEuler_fromQuaternion: pure 90deg yaw -> (0,0,90) ---\n");
    float c = cosf((float)M_PI / 4.0f);
    float s = sinf((float)M_PI / 4.0f);
    XsQuaternion q = { c, 0.0f, 0.0f, s };
    XsEuler e;
    XsEuler_fromQuaternion(&e, &q);
    assert_euler_close(0.0f, 0.0f, 90.0f, &e, 1e-3f, "yaw90");
}

static void test_euler_negative_roll(void) {
    printf("\n--- XsEuler_fromQuaternion: -45deg roll -> (-45,0,0) ---\n");
    float c = cosf((float)(-M_PI / 8.0));   /* cos(-22.5deg) */
    float s = sinf((float)(-M_PI / 8.0));   /* sin(-22.5deg) */
    XsQuaternion q = { c, s, 0.0f, 0.0f };
    XsEuler e;
    XsEuler_fromQuaternion(&e, &q);
    assert_euler_close(-45.0f, 0.0f, 0.0f, &e, 1e-3f, "roll-45");
}

/* ---- main ---- */

int main(void) {
    printf("=== xsmath Test Suite ===\n");

    test_multiply_identity_left();
    test_multiply_identity_right();
    test_multiply_basis_units();
    test_multiply_yaw90_squared();
    test_multiply_aliasing();

    test_euler_identity();
    test_euler_pure_roll_90();
    test_euler_pure_pitch_30();
    test_euler_pure_yaw_90();
    test_euler_negative_roll();

    printf("\n=== Test Results ===\n");
    printf("Passed: %d/%d\n", g_tests_passed, g_tests_total);
    if (g_tests_passed == g_tests_total) {
        printf("All tests PASSED!\n");
        return 0;
    }
    printf("Some tests FAILED!\n");
    return 1;
}
