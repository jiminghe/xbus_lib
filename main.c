#define _POSIX_C_SOURCE 200809L  /* nanosleep, clock_gettime */

#include "serial_reader.h"
#include "xbus/xbus.h"
#include "xbus/xbus_message_id.h"
#include "xbus/xbus_parser.h"
#include "xsdevice_def.h"
#include "xsmath/xsquaternion.h"
#include "xsmath/xseuler.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Mount presets (mirrors MOUNT_PRESETS in the Python script) ---- */

typedef struct { char axis; int deg; } MountStep;

static const MountStep PRESET_1[] = { {'z',  90}, {'y', 180} };
static const MountStep PRESET_2[] = { {'z', 180}, {'y',  90} };
static const MountStep PRESET_3[] = { {'z',   0}, {'y', -90} };

static const MountStep* g_mount = PRESET_1;
static size_t           g_mount_n = sizeof(PRESET_1) / sizeof(PRESET_1[0]);

static int select_preset(int n) {
    switch (n) {
    case 1: g_mount = PRESET_1; g_mount_n = sizeof(PRESET_1) / sizeof(*PRESET_1); return 1;
    case 2: g_mount = PRESET_2; g_mount_n = sizeof(PRESET_2) / sizeof(*PRESET_2); return 1;
    case 3: g_mount = PRESET_3; g_mount_n = sizeof(PRESET_3) / sizeof(*PRESET_3); return 1;
    default: return 0;
    }
}

/* ---- Tunables (mirrors OrientationCorrector class attributes) ---- */

#define VRU_PROFILE      1   /* yaw is reset every run (VRU / AHRS-no-mag) */
#define WRITE_TO_DEVICE  1   /* persist q_rotSensor/q_rotLocal at end */
#define WARMUP_MS        5000
#define RUN_DURATION_MS  10000
#define ACK_TIMEOUT_MS   1000

/* ---- Time + sleep helpers ---- */

static uint64_t mono_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

static void sleep_ms(int ms) {
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* ---- Xbus byte-stream parser (same shape used by the demo app) ---- */

#define PARSE_BUF_SIZE 1024

typedef struct {
    int     sync;
    uint8_t buf[PARSE_BUF_SIZE];
    size_t  len;
    size_t  expected;
} XbusParserState;

static void parser_reset(XbusParserState* p) {
    p->sync = 0;
    p->len = 0;
    p->expected = 0;
}

static const uint8_t* parser_feed_byte(XbusParserState* p, uint8_t b) {
    if (p->sync == 0) {
        if (b == XBUS_PREAMBLE) {
            p->buf[0] = b; p->len = 1; p->expected = 0; p->sync = 1;
        }
        return NULL;
    }
    if (p->len < PARSE_BUF_SIZE) p->buf[p->len++] = b;
    if (p->len >= 4 && p->expected == 0) {
        int raw = xbus_get_raw_length(p->buf);
        if (raw < 5 || raw > (int)PARSE_BUF_SIZE) { parser_reset(p); return NULL; }
        p->expected = (size_t)raw;
    }
    if (p->expected > 0 && p->len >= p->expected) {
        int ok = xbus_verify_checksum(p->buf);
#ifdef DEBUG
        fprintf(stderr, "[DBG RX] %zu bytes%s:",
                p->expected, ok ? "" : " (BAD CKSUM)");
        for (size_t i = 0; i < p->expected; i++) fprintf(stderr, " %02X", p->buf[i]);
        fputc('\n', stderr);
#endif
        const uint8_t* msg = ok ? p->buf : NULL;
        parser_reset(p);
        return msg;
    }
    if (p->len >= PARSE_BUF_SIZE - 1) parser_reset(p);
    return NULL;
}

/* ---- Synchronous wait helper: block until a message with `target_mid`
       arrives, or the timeout elapses. Other messages are silently dropped. */

static int wait_for_mid(SerialReader* port, XbusParserState* parser,
                        int target_mid, int timeout_ms) {
    uint64_t deadline = mono_ms() + (uint64_t)timeout_ms;
    while (mono_ms() < deadline) {
        uint8_t rx[256];
        int n = serial_reader_read_available(port, rx, sizeof(rx));
        for (int i = 0; i < n; i++) {
            const uint8_t* msg = parser_feed_byte(parser, rx[i]);
            if (!msg) continue;
            int mid = xbus_get_message_id(msg);
            if (mid == target_mid) return 1;
            if (mid == XMID_Error) return 0;
        }
        sleep_ms(2);
    }
    return 0;
}

/* ---- Quaternion conversion + small inline helpers ---- */

static XsQuaternion to_xs(const Quaternion* q) {
    XsQuaternion r = { q->q0, q->q1, q->q2, q->q3 };
    return r;
}

static Quaternion to_xb(const XsQuaternion* q) {
    Quaternion r = { q->w, q->x, q->y, q->z };
    return r;
}

static XsQuaternion quat_conjugate(XsQuaternion q) {
    XsQuaternion r = { q.w, -q.x, -q.y, -q.z };
    return r;
}

static XsQuaternion quat_axis_deg(char axis, int deg) {
    float half = (float)deg * (float)M_PI / 360.0f;
    float c = cosf(half), s = sinf(half);
    XsQuaternion r = { c, 0.0f, 0.0f, 0.0f };
    switch (axis) {
    case 'x': r.x = s; break;
    case 'y': r.y = s; break;
    case 'z': r.z = s; break;
    }
    return r;
}

static void write_float_be(uint8_t* dst, float v) {
    uint32_t u; memcpy(&u, &v, 4);
    dst[0] = (uint8_t)(u >> 24);
    dst[1] = (uint8_t)(u >> 16);
    dst[2] = (uint8_t)(u >>  8);
    dst[3] = (uint8_t)(u);
}

/* Build the wire bytes for a SetAlignmentRotation(quaternion) message and
   format them as space-separated hex — matches the Python print output. */
static void format_alignment_hex(char* out, size_t out_size,
                                 uint8_t frame, const XsQuaternion* q) {
    uint8_t msg[64];
    uint8_t payload[1 + 16];
    payload[0] = frame;
    write_float_be(&payload[1],  q->w);
    write_float_be(&payload[5],  q->x);
    write_float_be(&payload[9],  q->y);
    write_float_be(&payload[13], q->z);

    xbus_create_message(msg, XBUS_MASTERDEVICE, XMID_SetAlignmentRotation,
                        (uint16_t)sizeof(payload));
    memcpy(xbus_get_pointer_to_payload(msg), payload, sizeof(payload));
    xbus_insert_checksum(msg);

    int raw_len = xbus_get_raw_length(msg);
    size_t pos = 0;
    for (int i = 0; i < raw_len && pos + 4 < out_size; i++) {
        int w = snprintf(out + pos, out_size - pos, i == 0 ? "%02X" : " %02X", msg[i]);
        if (w < 0) break;
        pos += (size_t)w;
    }
    out[pos < out_size ? pos : out_size - 1] = '\0';
}

/* ---- Orientation corrector (mirrors the Python OrientationCorrector class) ---- */

typedef struct {
    int          initialized;
    XsQuaternion q_rot_sensor;
    XsQuaternion q_rot_local;
} OrientationCorrector;

static XsQuaternion build_mount_quaternion(void) {
    XsQuaternion q = { 1.0f, 0.0f, 0.0f, 0.0f };
    for (size_t i = 0; i < g_mount_n; i++) {
        XsQuaternion step = quat_axis_deg(g_mount[i].axis, g_mount[i].deg);
        XsQuaternion tmp;
        XsQuaternion_multiply(&step, &q, &tmp);  /* extrinsic: pre-multiply */
        q = tmp;
    }
    return q;
}

static void corrector_initialize(OrientationCorrector* oc, const XsQuaternion* q_raw) {
    XsQuaternion q_mount   = build_mount_quaternion();
    XsQuaternion q_unmount = quat_conjugate(q_mount);

    XsEuler raw_e; XsEuler_fromQuaternion(&raw_e, q_raw);
    printf("Initial raw orientation - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           (double)raw_e.roll, (double)raw_e.pitch, (double)raw_e.yaw);

    XsQuaternion q_natural;
    XsQuaternion_multiply(q_raw, &q_unmount, &q_natural);
    XsEuler nat_e; XsEuler_fromQuaternion(&nat_e, &q_natural);
    printf("Natural-frame estimate - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
           (double)nat_e.roll, (double)nat_e.pitch, (double)nat_e.yaw);

    /* Target: pure yaw at the natural-frame yaw (zero roll, zero pitch). */
    float yaw_rad = nat_e.yaw * (float)M_PI / 180.0f;
    XsQuaternion q_target = { cosf(yaw_rad * 0.5f), 0.0f, 0.0f, sinf(yaw_rad * 0.5f) };

    XsQuaternion q_raw_inv = quat_conjugate(*q_raw);
    XsQuaternion_multiply(&q_raw_inv, &q_target, &oc->q_rot_sensor);

    if (VRU_PROFILE) {
        /* Sum of explicit z-rotation degrees (extrinsic Z's commute,
           X/Y rotations don't contribute to world-Z yaw). */
        int yaw_sum_deg = 0;
        for (size_t i = 0; i < g_mount_n; i++) {
            if (g_mount[i].axis == 'z') yaw_sum_deg += g_mount[i].deg;
        }
        float my = (float)yaw_sum_deg * (float)M_PI / 180.0f;
        oc->q_rot_local.w = cosf(my * 0.5f);
        oc->q_rot_local.x = 0.0f;
        oc->q_rot_local.y = 0.0f;
        oc->q_rot_local.z = sinf(my * 0.5f);

        /* Warn if body X is nearly vertical — VRU yaw init becomes unstable. */
        float body_x_world_z = 2.0f * (q_mount.x * q_mount.z - q_mount.w * q_mount.y);
        if (fabsf(body_x_world_z) > 0.7f) {
            printf("\n");
            for (int i = 0; i < 70; i++) putchar('!'); putchar('\n');
            printf("  WARNING: this mounting tilts body X nearly vertical\n");
            printf("           (body_X . world_Z = %+.3f).\n", (double)body_x_world_z);
            printf("           VRU yaw initialization is unstable in this pose -\n");
            printf("           expect random yaw drift across power cycles.\n");
            printf("           Use NorthReference/FixedMagRef, or change the mounting.\n");
            for (int i = 0; i < 70; i++) putchar('!'); putchar('\n');
            printf("\n");
        }
    } else {
        XsQuaternion id = { 1.0f, 0.0f, 0.0f, 0.0f };
        oc->q_rot_local = id;
    }

    oc->initialized = 1;

    printf("Orientation correction initialized with:\n");
    printf("q_rotSensor: [%.7f, %.7f, %.7f, %.7f]\n",
           (double)oc->q_rot_sensor.w, (double)oc->q_rot_sensor.x,
           (double)oc->q_rot_sensor.y, (double)oc->q_rot_sensor.z);
    {
        char hex[128];
        format_alignment_hex(hex, sizeof(hex), 0, &oc->q_rot_sensor);
        printf("RotSensor HEX Command:\n%s\n", hex);
    }
    if (VRU_PROFILE) {
        printf("q_rotLocal:  [%.7f, %.7f, %.7f, %.7f]\n",
               (double)oc->q_rot_local.w, (double)oc->q_rot_local.x,
               (double)oc->q_rot_local.y, (double)oc->q_rot_local.z);
        char hex[128];
        format_alignment_hex(hex, sizeof(hex), 1, &oc->q_rot_local);
        printf("RotLocal HEX Command:\n%s\n", hex);
    }
    fflush(stdout);
}

/* corrected = q_rotLocal * (q_raw * q_rotSensor) */
static XsQuaternion corrector_apply(const OrientationCorrector* oc, const XsQuaternion* q_raw) {
    XsQuaternion partial, out;
    XsQuaternion_multiply(q_raw, &oc->q_rot_sensor, &partial);
    XsQuaternion_multiply(&oc->q_rot_local, &partial, &out);
    return out;
}

/* ---- main ---- */

static void print_mount(int preset_idx) {
    printf("Using mounting preset %d: [", preset_idx);
    for (size_t i = 0; i < g_mount_n; i++) {
        printf("%s('%c', %d)", i ? ", " : "", g_mount[i].axis, g_mount[i].deg);
    }
    printf("]\n");
}

int main(int argc, char** argv) {
    int preset = 1;
    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--preset-pose") == 0 || strcmp(argv[i], "-p") == 0)
            && i + 1 < argc) {
            preset = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [--preset-pose N]\n"
                   "  N=1 (default): z90, y180\n"
                   "  N=2          : z180, y90\n"
                   "  N=3          : y-90\n", argv[0]);
            return 0;
        }
    }
    if (!select_preset(preset)) {
        fprintf(stderr, "Invalid preset %d (must be 1, 2, or 3).\n", preset);
        return 1;
    }
    print_mount(preset);

    SerialReader port;
    serial_reader_init(&port);
    if (!serial_reader_open(&port, "/dev/ttyUSB0", 115200, 8, 'N', 1)) {
        fprintf(stderr, "Failed to open /dev/ttyUSB0: %s\n", serial_reader_last_error(&port));
        return 1;
    }

    XbusParserState parser;
    parser_reset(&parser);

    /* ----- Configure: enter config mode, reset alignments, set Quaternion output ----- */
    printf("Putting device into configuration mode...\n");
    gotoConfig(&port);
    if (!wait_for_mid(&port, &parser, XMID_GotoConfigAck, ACK_TIMEOUT_MS)) {
        fprintf(stderr, "Could not put device into configuration mode. Aborting.\n");
        serial_reader_close(&port); return 1;
    }

    {
        printf("Resetting RotSensor and RotLocal to identity...\n");
        Quaternion id = { 1.0f, 0.0f, 0.0f, 0.0f };
        setAlignmentRotationQuaternion(&port, SRM_ROTSENSOR, &id);
        if (!wait_for_mid(&port, &parser, XMID_SetAlignmentRotationAck, ACK_TIMEOUT_MS)) {
            fprintf(stderr, "Could not reset RotSensor. Aborting.\n");
            serial_reader_close(&port); return 1;
        }
        setAlignmentRotationQuaternion(&port, SRM_ROTLOCAL, &id);
        if (!wait_for_mid(&port, &parser, XMID_SetAlignmentRotationAck, ACK_TIMEOUT_MS)) {
            fprintf(stderr, "Could not reset RotLocal. Aborting.\n");
            serial_reader_close(&port); return 1;
        }
    }

    printf("Putting device into measurement mode...\n");
    gotoMeasurement(&port);
    if (!wait_for_mid(&port, &parser, XMID_GotoMeasurementAck, ACK_TIMEOUT_MS)) {
        fprintf(stderr, "Could not put device into measurement mode. Aborting.\n");
        serial_reader_close(&port); return 1;
    }

    printf("Main loop. Recording data for %d seconds.\n", RUN_DURATION_MS / 1000);
    for (int i = 0; i < 70; i++) putchar('='); putchar('\n');
    printf("  >>> DO NOT MOVE THE SENSOR for the next 6 seconds <<<\n");
    printf("  The filter is converging during a %d s warm-up; the\n", WARMUP_MS / 1000);
    printf("  calibration frame is captured immediately after and must be at rest.\n");
    for (int i = 0; i < 70; i++) putchar('='); putchar('\n');
    fflush(stdout);

    /* ----- Stream loop: warm up, then capture + correct ----- */
    OrientationCorrector oc;
    memset(&oc, 0, sizeof(oc));

    uint64_t t0 = mono_ms();
    while (mono_ms() - t0 <= (uint64_t)RUN_DURATION_MS) {
        uint8_t rx[256];
        int n = serial_reader_read_available(&port, rx, sizeof(rx));
        for (int i = 0; i < n; i++) {
            const uint8_t* msg = parser_feed_byte(&parser, rx[i]);
            if (!msg) continue;
            if (xbus_get_message_id(msg) != XMID_MtData2) continue;

            uint64_t elapsed = mono_ms() - t0;
            if (elapsed < (uint64_t)WARMUP_MS) continue;  /* drain during warm-up */

            SensorData sd;
            if (!xbus_parse_mt_data2(msg, &sd) || !sd.has_quaternion) continue;

            XsQuaternion q_raw = to_xs(&sd.quaternion);
            if (!oc.initialized) {
                corrector_initialize(&oc, &q_raw);
                continue;
            }

            XsQuaternion q_cor = corrector_apply(&oc, &q_raw);
            XsEuler raw_e, cor_e;
            XsEuler_fromQuaternion(&raw_e, &q_raw);
            XsEuler_fromQuaternion(&cor_e, &q_cor);
            printf("\rRaw R/P/Y: %7.2f %7.2f %7.2f | Cor R/P/Y: %7.2f %7.2f %7.2f",
                   (double)raw_e.roll, (double)raw_e.pitch, (double)raw_e.yaw,
                   (double)cor_e.roll, (double)cor_e.pitch, (double)cor_e.yaw);
            fflush(stdout);
        }
        sleep_ms(1);
    }
    printf("\n");

    /* ----- Persist alignment back to the device ----- */
    printf("Returning device to configuration mode...\n");
    gotoConfig(&port);
    if (!wait_for_mid(&port, &parser, XMID_GotoConfigAck, ACK_TIMEOUT_MS)) {
        fprintf(stderr, "Could not return device to configuration mode. Aborting.\n");
        serial_reader_close(&port); return 1;
    }

    if (WRITE_TO_DEVICE && oc.initialized) {
        printf("Writing q_rotSensor to device...\n");
        Quaternion qs = to_xb(&oc.q_rot_sensor);
        setAlignmentRotationQuaternion(&port, SRM_ROTSENSOR, &qs);
        if (!wait_for_mid(&port, &parser, XMID_SetAlignmentRotationAck, ACK_TIMEOUT_MS)) {
            fprintf(stderr, "Failed to write RotSensor. Aborting.\n");
            serial_reader_close(&port); return 1;
        }
        if (VRU_PROFILE) {
            printf("Writing q_rotLocal to device...\n");
            Quaternion ql = to_xb(&oc.q_rot_local);
            setAlignmentRotationQuaternion(&port, SRM_ROTLOCAL, &ql);
            if (!wait_for_mid(&port, &parser, XMID_SetAlignmentRotationAck, ACK_TIMEOUT_MS)) {
                fprintf(stderr, "Failed to write RotLocal. Aborting.\n");
                serial_reader_close(&port); return 1;
            }
        }
    }

    serial_reader_close(&port);
    printf("Successful exit.\n");
    return 0;
}
