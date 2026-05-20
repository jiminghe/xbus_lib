# xbus_lib — Linux/C library and tools for Xsens MTi sensors

A pure-C library and command-line tool for talking to Xsens MTi motion
trackers over a UART/USB serial port on Linux. The headline application
is `mti_mount_calibrate`, which computes and writes the **RotSensor** /
**RotLocal** alignment quaternions when the IMU is mounted in a non-natural
orientation.

The codebase is intentionally small and embedded-friendly: no threads, no
callbacks, no DLLs, no C++. The shape mirrors Xsens's own embedded
example (`embedded_examples/example_mti1_i2c_spi_receive_measurement_data`):
a polling main loop that dispatches both serial bytes and keyboard events
through a single state machine.

## Layout

```
xbus_lib/
├── build.sh                 # build / clean / rebuild helper
├── CMakeLists.txt
├── main.c                   # mti_mount_calibrate application
├── serial_reader.{h,c}      # thin POSIX termios wrapper
├── xsdevice_def.{h,c}       # high-level Xsens commands (gotoConfig, …)
├── xbus/
│   ├── xbus.{h,c}           # Xbus frame helpers (preamble, length, checksum)
│   ├── xbus_message_id.h    # XMID_* message-id constants
│   └── xbus_parser.{h,c}    # MtData2 XDI decoder + message formatter
├── xsmath/
│   ├── xsquaternion.{h,c}   # XsQuaternion + XsQuaternion_multiply
│   └── xseuler.{h,c}        # XsEuler + XsEuler_fromQuaternion
└── test/
    ├── CMakeLists.txt
    ├── test_xbus_parser.c   # 65 cases: MtData2/XDI parsing, FP1632, framing
    └── test_xsmath.c        # 51 cases: quaternion multiply, euler conversion
```

## Requirements

- Linux (developed on Ubuntu 24.04)
- GCC or Clang with C11 support
- CMake ≥ 3.10
- User in the `dialout` group (for `/dev/ttyUSB*` permission)
- An Xsens MTi 1/600/etc. accessible on `/dev/ttyUSB0`

```bash
sudo usermod -aG dialout $USER     # then log out / log in
```

## Quick start

```bash
git clone <repo-url>
cd xbus_lib

./build.sh                  # builds build/mti_mount_calibrate + tests

./build/mti_mount_calibrate --preset-pose 1
```

## Building

The `build.sh` helper wraps CMake for both the main project and the test
suite.

| Command               | Action                                              |
|-----------------------|-----------------------------------------------------|
| `./build.sh`          | Same as `./build.sh build`.                         |
| `./build.sh build`    | Incremental build of `build/` and `test/build/`.    |
| `./build.sh clean`    | Remove `build/` and `test/build/`.                  |
| `./build.sh rebuild`  | `clean` followed by `build`.                        |
| `./build.sh help`     | Print the usage header.                             |

The script uses `set -euo pipefail`, so any underlying CMake or compiler
failure aborts the run with a non-zero exit status.

Manual CMake invocation, if preferred:

```bash
cmake -S . -B build
cmake --build build -j

cmake -S test -B test/build
cmake --build test/build -j
```

### Tests

```bash
ctest --test-dir test/build
```

Two executables, both currently passing 116/116 assertions:

- `xbus_parser_test` — MtData2 payload decoding for every XDI we handle
  (PacketCounter, SampleTimeFine, EulerAngles, Quaternion, LatLon,
  AltitudeEllipsoid, VelocityXYZ, UtcTime, BarometricPressure,
  StatusWord), plus FP16.32 conversion and bad-frame rejection.
- `xsmath_test` — `XsQuaternion_multiply` (identity, Hamilton basis,
  aliasing) and `XsEuler_fromQuaternion` (identity, pure roll/pitch/yaw,
  sign convention).

## Using `mti_mount_calibrate`

The tool is a port of `mti_receive_data_xda.py`. It expects a known
mounting orientation, captures a stable IMU reading, and writes the
appropriate alignment quaternions to non-volatile memory on the device
so subsequent runs report angles in the natural (zero-tilt) frame.

```
./build/mti_mount_calibrate [--preset-pose N]
```

| Preset | Mount steps (extrinsic)  | Use when …                              |
|--------|--------------------------|-----------------------------------------|
| `1`    | `z = +90°, y = +180°`    | default; sensor rotated 90° around Z then flipped over Y |
| `2`    | `z = +180°, y = +90°`    |                                         |
| `3`    | `y = -90°`               |                                         |

### What it does

1. Opens `/dev/ttyUSB0` at 115200 8N1.
2. Sends `GotoConfig`, waits for `GotoConfigAck`.
3. Resets `RotSensor` and `RotLocal` to identity so we read the unaltered
   sensor output.
4. Configures the output to Quaternion @ 100 Hz.
5. Sends `GotoMeasurement`, drains frames for **5 s** while the filter
   converges (sensor must be at rest).
6. Captures the next frame and computes:
   - `q_rotSensor` such that `q_raw · q_rotSensor` has zero roll / zero
     pitch and reports the natural-frame yaw.
   - `q_rotLocal` = the sum of explicit Z-rotation degrees in the
     mounting (VRU/AHRS-no-mag profiles only).
7. Streams Raw vs Corrected angles for another ~5 s on stdout.
8. Returns to config mode and writes `q_rotSensor`/`q_rotLocal` to the
   device via `SetAlignmentRotation`.

### Example output

```
Using mounting preset 1: [('z', 90), ('y', 180)]
Putting device into configuration mode...
Resetting RotSensor and RotLocal to identity...
Putting device into measurement mode...
Main loop. Recording data for 10 seconds.
======================================================================
  >>> DO NOT MOVE THE SENSOR for the next 6 seconds <<<
  …
======================================================================
Initial raw orientation - Roll: -7.97, Pitch: -89.16, Yaw: -60.76
Natural-frame estimate - Roll: 90.83, Pitch: 0.12, Yaw: 21.28
Orientation correction initialized with:
q_rotSensor: [0.5041357, 0.4968571, 0.4958298, 0.5031233]
RotSensor HEX Command:
FA FF EC 11 00 3F 01 0F 0A 3E FE 64 0D 3E FD DD 68 3F 00 CC B0 C3
q_rotLocal:  [0.7071068, 0.0000000, 0.0000000, 0.7071068]
RotLocal HEX Command:
FA FF EC 11 01 3F 35 04 F3 00 00 00 00 00 00 00 00 3F 35 04 F3 2D
Raw R/P/Y:   -8.22  -89.14  -60.52 | Cor R/P/Y:    0.01    0.01  111.26
Returning device to configuration mode...
Writing q_rotSensor to device...
Writing q_rotLocal to device...
Successful exit.
```

### Configuration knobs

These are `#define`s near the top of `main.c`; rebuild after changing.

| Symbol            | Default | Meaning                                              |
|-------------------|---------|------------------------------------------------------|
| `VRU_PROFILE`     | `1`     | Set to `0` for NorthReference/FixedMagRef filters.   |
| `WRITE_TO_DEVICE` | `1`     | Set to `0` to just print the HEX command, no write.  |
| `WARMUP_MS`       | `5000`  | Filter-convergence drain time.                       |
| `RUN_DURATION_MS` | `10000` | Total capture run length.                            |
| `ACK_TIMEOUT_MS`  | `1000`  | Per-command ack wait.                                |

The port path (`/dev/ttyUSB0`) and baud (`115200`) are likewise constants
near the top of `main()`.

## Debug Mode

A compile-time `DEBUG` switch dumps every Xbus message sent or received
as hex, gated by `#ifdef DEBUG` blocks in `xsdevice_def.c` (TX) and
`main.c::parser_feed_byte` (RX). Output goes to **stderr** so live
stdout streaming is unaffected.

Enable through the `DEBUG_HEX` CMake option — directly, or via the build
script's environment variable:

```bash
DEBUG_HEX=1 ./build.sh rebuild

# or manually:
cmake -S . -B build -DDEBUG_HEX=ON
cmake --build build -j
```

When the flag is on, CMake prints:

```
-- DEBUG_HEX enabled — raw TX/RX hex will be printed to stderr
```

Typical handshake trace:

```
[DBG TX] 5 bytes: FA FF 30 00 D1                  ← GotoConfig
[DBG RX] 5 bytes: FA FF 31 00 D0                  ← GotoConfigAck (MID+1)
[DBG TX] 22 bytes: FA FF EC 11 00 3F 80 …         ← SetAlignmentRotation
[DBG RX] 5 bytes: FA FF ED 00 14                  ← SetAlignmentRotationAck
```

Capture only the trace while still seeing the live calibration display:

```bash
./build/mti_mount_calibrate 2> hex.log
```

Plain (non-debug) builds compile the print statements out entirely — no
runtime overhead when `DEBUG_HEX` is OFF.

## API reference

All headers expose plain C functions (`extern "C"` guarded for C++ use).

### `xbus/xbus.h` — frame helpers

```c
bool xbus_check_preamble        (const uint8_t* msg);
int  xbus_get_message_id        (const uint8_t* msg);
int  xbus_get_payload_length    (const uint8_t* msg);
int  xbus_get_raw_length        (const uint8_t* msg);
void xbus_create_message        (uint8_t* msg, uint8_t bid, uint8_t mid, uint16_t len);
uint8_t* xbus_get_pointer_to_payload(uint8_t* msg);
void xbus_insert_checksum       (uint8_t* msg);
bool xbus_verify_checksum       (const uint8_t* msg);
```

### `xbus/xbus_parser.h` — MtData2 decoding

```c
typedef struct { float roll, pitch, yaw; } EulerAngles;
typedef struct { float q0, q1, q2, q3;   } Quaternion;
typedef struct { /* has_* flags + payload fields */ } SensorData;

bool xbus_parse_mt_data2 (const uint8_t* msg, SensorData* out);
int  xbus_format_message (const uint8_t* msg, char* buf, size_t bs);
```

XDIs decoded: `0x1020` PacketCounter, `0x1060` SampleTimeFine, `0x2010`
Quaternion, `0x2030` EulerAngles, `0x5042` LatLon (FP16.32),
`0x5022` AltitudeEllipsoid, `0xD012` VelocityXYZ, `0x1010` UtcTime,
`0x3010` BarometricPressure, `0xE020` StatusWord.

### `xsdevice_def.h` — device commands (fire-and-forget)

```c
typedef enum { SRM_ROTSENSOR = 0, SRM_ROTLOCAL = 1 } SetRotationMatrix;

bool gotoConfig                    (SerialReader* port);
bool gotoMeasurement               (SerialReader* port);
bool reqDid                        (SerialReader* port);
bool reqFwVersion                  (SerialReader* port);
bool setOutputConfiguration        (SerialReader* port,
                                    const XsOutputConfigItem* items, size_t count);
bool setAlignmentRotationQuaternion(SerialReader* port,
                                    SetRotationMatrix frame,
                                    const Quaternion* quat);
```

These build the Xbus frame and write it to the port. Reading the reply
(MID+1 ack) is the caller's responsibility — `main.c` does it with a
small synchronous `wait_for_mid()` helper.

### `serial_reader.h` — POSIX termios wrapper

```c
void serial_reader_init           (SerialReader* r);
bool serial_reader_open           (SerialReader* r, const char* port,
                                   int baud, int data_bits, char parity, int stop_bits);
void serial_reader_close          (SerialReader* r);
bool serial_reader_write          (SerialReader* r, const uint8_t* data, size_t len);
int  serial_reader_read_available (SerialReader* r, uint8_t* buf, size_t len);
bool serial_reader_flush          (SerialReader* r);
const char* serial_reader_last_error(const SerialReader* r);
```

### `xsmath/xsquaternion.h` + `xsmath/xseuler.h`

```c
typedef struct { float w, x, y, z; }            XsQuaternion;
typedef struct { float roll, pitch, yaw; }      XsEuler;  /* degrees */

void XsQuaternion_multiply  (const XsQuaternion* l, const XsQuaternion* r,
                             XsQuaternion* dest);   /* Hamilton product */
void XsEuler_fromQuaternion (XsEuler* dest, const XsQuaternion* q);
```

## Supported message IDs

| MID  | Name                       | Direction         | Reply (MID+1)            |
|------|----------------------------|-------------------|--------------------------|
| 0x00 | XMID_ReqDid                | host → device     | XMID_DeviceId (0x01)     |
| 0x10 | XMID_GotoMeasurement       | host → device     | XMID_GotoMeasurementAck (0x11) |
| 0x12 | XMID_ReqFirmwareRevision   | host → device     | XMID_FirmwareRevision (0x13) |
| 0x30 | XMID_GotoConfig            | host → device     | XMID_GotoConfigAck (0x31) |
| 0x36 | XMID_MtData2               | device → host     | —                        |
| 0x42 | XMID_Error                 | device → host     | —                        |
| 0xC0 | XMID_SetOutputConfig       | host → device     | XMID_OutputConfig (0xC1) |
| 0xEC | XMID_SetAlignmentRotation  | host → device     | XMID_SetAlignmentRotationAck (0xED) |

## Troubleshooting

**`Failed to open /dev/ttyUSB0: Permission denied`** — your user is not
in the `dialout` group; see Requirements above.

**`Failed to open /dev/ttyUSB0: Device or resource busy`** — another
process (e.g. MT Manager via virtualization, a previous run that did not
exit cleanly) holds the port. Identify it with `fuser /dev/ttyUSB0`.

**`Could not put device into configuration mode. Aborting.`** — the
device is sending data but did not ack within `ACK_TIMEOUT_MS`. Verify
115200 baud, that the device is powered, and try re-plugging the USB
cable. Enable `DEBUG_HEX` to inspect the raw traffic.

**Checksum errors on RX** — usually electrical: bad cable, ESD, a USB
hub interfering. With `DEBUG_HEX` on, `[DBG RX] N bytes (BAD CKSUM): …`
lines show the offending frames.

**Bad calibration result** — the sensor must be **stationary** during
the warm-up; movement during the 5–6 s after `GotoMeasurement` produces
a biased `q_rotSensor`. Re-run with the sensor flat and still.

## Acknowledgments

- Xsens / Movella Xbus protocol specification and embedded example code.
- Mount-calibration algorithm ported from
  `mti_receive_data_xda.py`.
