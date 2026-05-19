#define _POSIX_C_SOURCE 200809L  /* nanosleep */

#include "serial_reader.h"
#include "xbus/xbus.h"
#include "xbus/xbus_message_id.h"
#include "xbus/xbus_parser.h"
#include "xsdevice_def.h"

#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ---- Xbus byte-stream parser (polled, single-threaded) ---- */

#define PARSE_BUF_SIZE 1024

typedef struct {
    int     sync;            /* 0 = waiting preamble, 1 = reading message */
    uint8_t buf[PARSE_BUF_SIZE];
    size_t  len;
    size_t  expected;
} XbusParserState;

static void parser_reset(XbusParserState* p) {
    p->sync = 0;
    p->len = 0;
    p->expected = 0;
}

/* Returns the message pointer once a complete, checksum-verified frame is in
   p->buf; NULL otherwise. */
static const uint8_t* parser_feed_byte(XbusParserState* p, uint8_t b) {
    if (p->sync == 0) {
        if (b == XBUS_PREAMBLE) {
            p->buf[0] = b;
            p->len = 1;
            p->expected = 0;
            p->sync = 1;
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
        const uint8_t* msg = NULL;
        if (xbus_verify_checksum(p->buf)) msg = p->buf;
        parser_reset(p);
        return msg;
    }

    if (p->len >= PARSE_BUF_SIZE - 1) parser_reset(p);
    return NULL;
}

/* ---- Application state machine (mirrors application.cpp from the embedded SDK) ---- */

typedef enum {
    EVT_Start,
    EVT_GotoConfig,
    EVT_GotoMeasurement,
    EVT_RequestDeviceId,
    EVT_RequestFwVersion,
    EVT_XbusMessage
} Event;

typedef enum {
    STATE_Idle,
    STATE_WaitForConfigAck,
    STATE_WaitForDeviceId,
    STATE_WaitForFwRevision,
    STATE_WaitForOutputConfigAck,
    STATE_Ready
} AppState;

/* Module state — equivalent to the SDK's Application member variables. */
static SerialReader g_port;
static AppState     g_state = STATE_Idle;

static void print_help(void) {
    printf("\n");
    printf("(h) help:     Print this help text\n");
    printf("(c) config:   Goto config mode\n");
    printf("(m) measure:  Goto measurement mode\n");
    printf("(d) deviceid: Request device id\n");
    printf("(f) fwrev:    Request firmware revision\n");
    printf("(q) quit\n");
    printf("\n");
    fflush(stdout);
}

static void handle_event(Event event, const uint8_t* data) {
    switch (g_state) {
    case STATE_Idle:
        if (event == EVT_Start) {
            gotoConfig(&g_port);
            g_state = STATE_WaitForConfigAck;
        }
        break;

    case STATE_WaitForConfigAck:
        if (event == EVT_XbusMessage && xbus_get_message_id(data) == XMID_GotoConfigAck) {
            printf("Got XMID_GotoConfigAck\n");
            reqDid(&g_port);
            g_state = STATE_WaitForDeviceId;
        }
        break;

    case STATE_WaitForDeviceId:
        if (event == EVT_XbusMessage && xbus_get_message_id(data) == XMID_DeviceId) {
            printf("Got DeviceId\n");
            reqFwVersion(&g_port);
            g_state = STATE_WaitForFwRevision;
        }
        break;

    case STATE_WaitForFwRevision:
        if (event == EVT_XbusMessage && xbus_get_message_id(data) == XMID_FirmwareRevision) {
            printf("Got firmware revision\n");
            /* Output: Euler angles (0x2030) @ 100 Hz. */
            XsOutputConfigItem cfg[] = { { 0x2030, 100 } };
            setOutputConfiguration(&g_port, cfg, 1);
            g_state = STATE_WaitForOutputConfigAck;
        }
        break;

    case STATE_WaitForOutputConfigAck:
        if (event == EVT_XbusMessage && xbus_get_message_id(data) == XMID_OutputConfig) {
            printf("Output configuration written to device\n");
            g_state = STATE_Ready;
            print_help();
        }
        break;

    case STATE_Ready:
        if      (event == EVT_GotoConfig)        gotoConfig(&g_port);
        else if (event == EVT_GotoMeasurement)   gotoMeasurement(&g_port);
        else if (event == EVT_RequestDeviceId)   reqDid(&g_port);
        else if (event == EVT_RequestFwVersion)  reqFwVersion(&g_port);
        else if (event == EVT_XbusMessage) {
            char text[256];
            xbus_format_message(data, text, sizeof(text));
            printf("%s\n", text);
        }
        fflush(stdout);
        break;
    }
}

/* ---- Stdin polling ---- */

static bool stdin_has_line(void) {
    struct pollfd pfd = { STDIN_FILENO, POLLIN, 0 };
    return poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN);
}

/* ---- main ---- */

int main(void) {
    printf("Xbus Serial Reader\n");
    printf("==================\n");

    serial_reader_init(&g_port);
    if (!serial_reader_open(&g_port, "/dev/ttyUSB0", 115200, 8, 'N', 1)) {
        fprintf(stderr, "Failed to open /dev/ttyUSB0: %s\n", serial_reader_last_error(&g_port));
        return 1;
    }
    printf("Serial port /dev/ttyUSB0 opened successfully at 115200 baud.\n");

    XbusParserState parser;
    parser_reset(&parser);

    handle_event(EVT_Start, NULL);

    int running = 1;
    while (running) {
        /* Pump serial bytes → parser → handle_event(EVT_XbusMessage). */
        uint8_t rx[256];
        int n = serial_reader_read_available(&g_port, rx, sizeof(rx));
        for (int i = 0; i < n; i++) {
            const uint8_t* msg = parser_feed_byte(&parser, rx[i]);
            if (msg) handle_event(EVT_XbusMessage, msg);
        }

        /* Pump keyboard → handle_event(EVT_xxx). */
        if (stdin_has_line()) {
            char line[64];
            if (fgets(line, sizeof(line), stdin)) {
                char c = line[0];
                if      (c == 'q' || c == 'Q') running = 0;
                else if (c == 'h' || c == 'H') print_help();
                else if (c == 'c')             handle_event(EVT_GotoConfig,       NULL);
                else if (c == 'm')             handle_event(EVT_GotoMeasurement,  NULL);
                else if (c == 'd')             handle_event(EVT_RequestDeviceId,  NULL);
                else if (c == 'f')             handle_event(EVT_RequestFwVersion, NULL);
            } else {
                clearerr(stdin);
            }
        }

        struct timespec ts = { 0, 1 * 1000 * 1000 }; /* 1 ms */
        nanosleep(&ts, NULL);
    }

    serial_reader_close(&g_port);
    printf("Stopped and closed serial port.\n");
    return 0;
}
