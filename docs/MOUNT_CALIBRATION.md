# MTi Mount-Calibration Protocol

This document is a self-contained reference for someone implementing the
RotSensor calibration of an Xsens MTi sensor from scratch.

It covers, in order:

1. The Xbus framing required to talk to the device.
2. The end-to-end command sequence and the exact bytes you must send.
3. The mathematics that produces `q_rotSensor` from one captured
   measurement frame.
4. The three predefined mount orientations the calibration tool supports
   and their pre-computed values.
5. A fully worked numeric example.

You do not need to read any source code to follow this document.

---

## 1. Xbus framing

Every byte that goes between the host and the MTi is wrapped in an Xbus
frame:

```
+----------+-----+-----+-----+---------+----------+
| 0xFA     | BID | MID | LEN | PAYLOAD | CHECKSUM |
+----------+-----+-----+-----+---------+----------+
  1 byte    1     1     1+    LEN bytes  1
```

| Field    | Width        | Value                                                   |
|----------|--------------|---------------------------------------------------------|
| Preamble | 1 byte       | always `0xFA`                                           |
| BID      | 1 byte       | bus id; for serial use `0xFF` (master device)           |
| MID      | 1 byte       | message id (see §3)                                     |
| LEN      | 1 or 3 bytes | payload length (see below)                              |
| PAYLOAD  | LEN bytes    | command-specific                                        |
| CHECKSUM | 1 byte       | see formula below                                       |

### 1.1 Length encoding

If the payload is shorter than 255 bytes, LEN is one byte holding the
length. Otherwise, LEN is three bytes: `0xFF` followed by the length as
a 16-bit big-endian value. All commands in this document have short
payloads (≤ 17 bytes), so LEN is always one byte.

### 1.2 Checksum

The checksum is set so that the sum of every byte from BID up to and
including the checksum equals 0 modulo 256:

```
checksum = (- (BID + MID + LEN + PAYLOAD[0] + ... + PAYLOAD[LEN-1])) & 0xFF
```

To verify a received frame, sum from BID through the checksum byte; the
result must be `0 (mod 256)`.

### 1.3 Endianness

All multi-byte integer fields are **big-endian**. All floats are IEEE-754
single-precision (4 bytes), also stored **big-endian**.

### 1.4 Reply convention

For every command MID, the device replies with MID + 1 (e.g. `0x30`
GotoConfig → `0x31` GotoConfigAck). On failure, the device sends
`XMID_Error = 0x42` instead, with a one-byte error code in the payload.

---

## 2. End-to-end command sequence

The calibration runs in three phases. All bytes below are exact: copy
them onto the wire as shown (with the BID byte `0xFF` for the master
device, and the checksum computed per §1.2).

### Phase A — prepare device

| # | Direction  | Command                                                      | Bytes                                                              |
|---|------------|--------------------------------------------------------------|--------------------------------------------------------------------|
| 1 | host → dev | GotoConfig (`0x30`)                                          | `FA FF 30 00 D1`                                                   |
| 2 | dev → host | GotoConfigAck (`0x31`)                                       | `FA FF 31 00 D0`                                                   |
| 3 | host → dev | SetAlignmentRotation, RotSensor = identity (`0xEC`, frame=0) | `FA FF EC 11 00 3F 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 45` |
| 4 | dev → host | SetAlignmentRotationAck (`0xED`)                             | `FA FF ED 00 14`                                                   |
| 5 | host → dev | SetOutputConfiguration: Quaternion @ 100 Hz (`0xC0`)         | `FA FF C0 04 20 10 00 64 C9`                                       |
| 6 | dev → host | OutputConfig echo (`0xC1`)                                   | length and contents vary; payload echoes the requested items       |
| 7 | host → dev | GotoMeasurement (`0x10`)                                     | `FA FF 10 00 F1`                                                   |
| 8 | dev → host | GotoMeasurementAck (`0x11`)                                  | `FA FF 11 00 F0`                                                   |

**Notes**
- Step 3 — identity quaternion `(w, x, y, z) = (1, 0, 0, 0)`. Its
  big-endian IEEE-754 bytes are `3F 80 00 00 00 00 00 00 00 00 00 00 00
  00 00 00`. The first payload byte (after LEN) is the *frame*:
  `0x00` = RotSensor.
- Step 5 — `SetOutputConfiguration` payload is a list of
  `(XDI:u16-be, frequency:u16-be)` tuples. `0x2010` = Quaternion XDI,
  `0x0064` = 100 Hz. You can extend this to multiple tuples (each 4
  bytes); the example shown requests a single tuple.

### Phase B — capture

After GotoMeasurement, the device starts streaming MtData2 frames
(MID `0x36`) at the configured rate. The host must:

1. **Discard** all frames received within the first `5000 ms` after the
   GotoMeasurementAck. The Kalman filter is converging; orientation is
   not yet trustworthy. *The sensor must be physically motionless
   throughout this window.*
2. After 5000 ms, take the first complete MtData2 frame and extract the
   quaternion (XDI `0x2010`, 12-byte payload section: four IEEE-754
   big-endian floats `w, x, y, z`). Call this `q_raw`.
3. Compute `q_rotSensor` from `q_raw` using §4.

MtData2 layout reminder (only the parts you need here):

```
FA FF 36 LEN  XDI:u16-be  SIZE:u8  DATA...  ... more (XDI,SIZE,DATA) tuples ...  CHECKSUM
```

The Quaternion XDI is `0x2010` with `SIZE = 0x10` (16 bytes = 4
big-endian floats).

### Phase C — persist alignment

| # | Direction  | Command                                                | Bytes                                                            |
|---|------------|--------------------------------------------------------|------------------------------------------------------------------|
| 1 | host → dev | GotoConfig (`0x30`)                                    | `FA FF 30 00 D1`                                                 |
| 2 | dev → host | GotoConfigAck (`0x31`)                                 | `FA FF 31 00 D0`                                                 |
| 3 | host → dev | SetAlignmentRotation, frame=0, payload = `q_rotSensor` | `FA FF EC 11 00 <w:4> <x:4> <y:4> <z:4> <chk>`                   |
| 4 | dev → host | SetAlignmentRotationAck (`0xED`)                       | `FA FF ED 00 14`                                                 |

After step 4 the device has stored the new alignment in non-volatile
memory; future power-ups will apply it automatically.

---

## 3. Message reference

The full table of commands used in this protocol:

| MID  | Name                       | Direction         | Reply             |
|------|----------------------------|-------------------|-------------------|
| 0x10 | GotoMeasurement            | host → device     | 0x11              |
| 0x30 | GotoConfig                 | host → device     | 0x31              |
| 0x36 | MtData2                    | device → host     | (no reply)        |
| 0x42 | Error                      | device → host     | (no reply)        |
| 0xC0 | SetOutputConfiguration     | host → device     | 0xC1 OutputConfig |
| 0xEC | SetAlignmentRotation       | host → device     | 0xED              |

All MIDs that take a payload follow the convention "reply MID = request
MID + 1". The Error message (`0x42`) is sent **instead of** the expected
ack when the device cannot satisfy a request.

---

## 4. Computing `q_rotSensor`

### 4.1 Quaternion conventions used in this document

`q = (w, x, y, z)`, where `w` is the scalar part.

- **Multiplication** (Hamilton product). `r = a ⊗ b`:
  ```
  r.w = a.w·b.w − a.x·b.x − a.y·b.y − a.z·b.z
  r.x = a.x·b.w + a.w·b.x − a.z·b.y + a.y·b.z
  r.y = a.y·b.w + a.z·b.x + a.w·b.y − a.x·b.z
  r.z = a.z·b.w − a.y·b.x + a.x·b.y + a.w·b.z
  ```
- **Conjugate / inverse for unit quaternions**:
  `q* = (w, −x, −y, −z)`.
- **Axis-angle constructor** (rotation of `θ` rad about an axis):
  ```
  q_x(θ) = (cos(θ/2), sin(θ/2), 0, 0)
  q_y(θ) = (cos(θ/2), 0, sin(θ/2), 0)
  q_z(θ) = (cos(θ/2), 0, 0, sin(θ/2))
  ```
- **Tait-Bryan ZYX Euler extraction** from a unit quaternion, in degrees:
  ```
  sinr = 2(w·x + y·z)
  cosr = 1 − 2(x² + y²)
  roll  = atan2(sinr, cosr) · 180/π

  sinp = 2(w·y − z·x)               (clamp to [−1, 1] before asin)
  pitch = asin(sinp) · 180/π

  siny = 2(w·z + x·y)
  cosy = 1 − 2(y² + z²)
  yaw   = atan2(siny, cosy) · 180/π
  ```

### 4.2 Build the mounting quaternion

A mount preset is an ordered list of `(axis, degrees)` pairs interpreted
as **extrinsic** rotations applied in order. Build `q_mount` by
composing axis quaternions, **pre-multiplying** at each step:

```
q_mount ← (1, 0, 0, 0)
for (axis_i, deg_i) in preset, in order:
    q_step ← axis quaternion for axis_i with angle deg_i
    q_mount ← q_step ⊗ q_mount         # pre-multiply
return q_mount
```

### 4.3 Estimate the natural-frame orientation

The raw quaternion `q_raw` carries the actual sensor pose at rest,
including the mounting. Undo the mounting to estimate what the
orientation would have been if the sensor were in its natural pose:

```
q_unmount = conjugate(q_mount)
q_natural = q_raw ⊗ q_unmount
```

Convert `q_natural` to Euler (§4.1). Call the yaw component
`yaw_natural` (in degrees).

The roll and pitch of `q_natural` should be near zero if the sensor is
truly flat and the mounting matches the preset. Large residuals point
to an off-level surface or a wrong preset.

### 4.4 Build the target orientation

The corrected sensor should report **zero roll, zero pitch, and the
natural-frame yaw**. Express that as a pure-yaw quaternion:

```
q_target = (cos(yaw_natural · π / 360),
            0,
            0,
            sin(yaw_natural · π / 360))
```

(The division by 360 is the half-angle plus deg-to-rad combined: `deg ·
π / 360 = (deg/2) · π / 180`.)

### 4.5 Solve for `q_rotSensor`

The device applies the correction as `q_corrected = q_raw ⊗ q_rotSensor`.
Setting `q_corrected = q_target` and solving:

```
q_rotSensor = conjugate(q_raw) ⊗ q_target
```

Write the four floats to the device as the payload of
SetAlignmentRotation with frame = `0x00` (RotSensor). See §2 phase C
for the exact frame layout.

---

## 5. The three predefined mount orientations

The calibration tool ships with three mount presets. The table lists
each preset's rotation sequence and the resulting `q_mount`.

| Preset | Rotation steps (extrinsic) | `q_mount` `(w, x, y, z)`                        |
|--------|----------------------------|-------------------------------------------------|
| 1      | z = +90°, then y = +180°   | `(0.0000000, 0.7071068, 0.7071068, 0.0000000)`  |
| 2      | z = +180°, then y = +90°   | `(0.0000000, 0.7071068, 0.0000000, 0.7071068)`  |
| 3      | y = −90°                   | `(0.7071068, 0.0000000, −0.7071068, 0.0000000)` |

`q_rotSensor` for each preset is **not** a fixed value — it depends on
the specific `q_raw` measured at runtime; see §4.5 and the example in
§6.

`q_mount` derivation for preset 1, by hand:

```
q_step₁ = q_z(+90°) = (cos45°, 0, 0, sin45°) = (0.7071068, 0, 0, 0.7071068)
q_mount = q_step₁ ⊗ (1, 0, 0, 0) = (0.7071068, 0, 0, 0.7071068)

q_step₂ = q_y(+180°) = (cos90°, 0, sin90°, 0) = (0, 0, 1, 0)
q_mount = q_step₂ ⊗ q_mount

       w =  0·0.7071 − 0·0 − 1·0 − 0·0.7071     = 0
       x =  0·0.7071 + 0·0 − 0·0 + 1·0.7071     = 0.7071068
       y =  1·0.7071 + 0·0 + 0·0 − 0·0.7071     = 0.7071068
       z =  0·0.7071 − 1·0 + 0·0 + 0·0.7071     = 0
       q_mount = (0, 0.7071068, 0.7071068, 0)
```

Identical method applies to presets 2 and 3.

---

## 6. Worked example (preset 1, idealised)

Assume a sensor mounted exactly per preset 1 (z=+90°, y=+180°) on a
perfectly level surface. The raw quaternion the device reports at rest
is the mounting quaternion itself:

```
q_raw = q_mount = (0, 0.7071068, 0.7071068, 0)
```

### Step 1 — unmount

```
q_unmount = conjugate(q_mount) = (0, −0.7071068, −0.7071068, 0)
```

### Step 2 — natural-frame estimate

```
q_natural = q_raw ⊗ q_unmount = q_mount ⊗ conjugate(q_mount) = (1, 0, 0, 0)
```

This is the identity — zero roll, zero pitch, zero yaw. Realistic
captures show small residuals from non-ideal mounting; the algorithm
absorbs them.

### Step 3 — natural-frame Euler

```
yaw_natural = 0°
```

### Step 4 — target

```
q_target = (cos 0°, 0, 0, sin 0°) = (1, 0, 0, 0)
```

### Step 5 — solve

```
q_rotSensor = conjugate(q_raw) ⊗ q_target
            = (0, −0.7071068, −0.7071068, 0) ⊗ (1, 0, 0, 0)
            = (0, −0.7071068, −0.7071068, 0)
```

### Step 6 — wire bytes for the SetAlignmentRotation write

IEEE-754 single-precision big-endian encodings of the constants used
below:

| Float       | Bytes       |
|-------------|-------------|
| `+1.0`      | `3F 80 00 00` |
| `0.0`       | `00 00 00 00` |
| `+0.7071068`| `3F 35 04 F3` |
| `−0.7071068`| `BF 35 04 F3` |

**Frame for RotSensor** (frame byte = `0x00`, payload =
`q_rotSensor = (0, −0.7071068, −0.7071068, 0)`):

```
FA FF EC 11 00 00 00 00 00 BF 35 04 F3 BF 35 04 F3 00 00 00 00 2E
```

Checksum derivation (sum of bytes from BID through last payload byte,
then negated mod 256):

```
sum = 0xFF + 0xEC + 0x11 + 0x00 + 0x00 + 0x00 + 0x00 + 0x00
    + 0xBF + 0x35 + 0x04 + 0xF3 + 0xBF + 0x35 + 0x04 + 0xF3
    + 0x00 + 0x00 + 0x00 + 0x00
    = 1490 = 0x5D2
sum mod 256 = 0xD2
checksum    = (256 − 0xD2) mod 256 = 0x2E
```

The frame goes through the SetAlignmentRotationAck path described in
§2 phase C.

---

## 7. Default operating parameters

These values are conventional and can be tuned, but unless you have a
reason to change them, use:

| Symbol              | Value     | Meaning                                                       |
|---------------------|-----------|---------------------------------------------------------------|
| Baud rate           | 115200    | UART baud                                                     |
| Frame format        | 8N1       | 8 data bits, no parity, 1 stop bit                            |
| Quaternion output   | 100 Hz    | rate written in step 5 of phase A                             |
| Warm-up duration    | 5000 ms   | drain interval before the calibration frame is captured       |
| Total capture time  | 10000 ms  | warm-up + monitoring; you can read corrected angles afterwards|
| Per-ack timeout     | 1000 ms   | host gives up if no ack/error arrives in this many ms         |

---

## 8. Error handling

- If at any point the device sends `0x42` (Error) instead of the
  expected ack, abort and surface the one-byte error code from the
  payload.
- If the ack does not arrive within the per-command timeout, abort.
  Common causes: device not in measurement mode (for `0xC0`), wrong
  baud rate, or the cable was disconnected.
- If any received frame's checksum (§1.2) does not match, discard the
  frame and continue parsing the byte stream — do not abort.
- The sensor **must remain motionless** for the duration of phase B's
  warm-up. Movement biases `q_raw` and produces a wrong
  `q_rotSensor`. If you suspect the sensor moved, restart the
  calibration.
