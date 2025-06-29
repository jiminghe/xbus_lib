# XBus Parser Tests

This directory contains unit tests for the XBus parser functionality.

## Structure

```
xbus_lib/
├── scripts/
│   ├── build_test.bat      # Test build script
│   └── clean_test.bat      # Test clean script
├── test/
│   ├── CMakeLists.txt      # CMake configuration for tests
│   ├── test_xbus_parser.cpp # Main test file
│   └── README.md           # This file
└── ... (other project files)
```

## Test Coverage

The test suite covers the following scenarios:

### 1. FP1632 Conversion Tests
- Tests the conversion of 6-byte FP16.32 fixed-point format to double
- Uses real data from sensor screenshots:
  - Latitude: `31.393166223541` (hex: `64 A6 8A A8 00 1F`)
  - Longitude: `121.229738174938` (hex: `3A D0 1E FC 00 79`)
  - Altitude: `56.714969451306` (hex: `B7 0B 3C EB 00 38`)

### 2. Complete MTData2 Message Tests
Tests parsing of a complete MTData2 message with all components:
- **PacketCounter** (0x1020): `2826`
- **SampleTimeFine** (0x1060): `12931224`
- **EulerAngles** (0x2030): Roll=`179.93°`, Pitch=`-1.15°`, Yaw=`-2.34°`
- **StatusWord** (0xE020): `0x00000002`
- **LatLon** (0x5042): Lat=`31.393166223541°`, Lon=`121.229738174938°`
- **AltitudeEllipsoid** (0x5022): `56.714969451306m`
- **VelocityXYZ** (0xD012): X=`-0.0215m/s`, Y=`0.0138m/s`, Z=`-0.0435m/s`

### 3. Individual Component Tests
- **Euler Angles Only**: Tests parsing of orientation data only
- **LatLon Only**: Tests GPS coordinate parsing
- **Velocity Only**: Tests velocity vector parsing

### 4. Error Handling Tests
- **Invalid Preamble**: Tests rejection of malformed messages
- **Wrong Message ID**: Tests rejection of non-MTData2 messages

## Building and Running Tests

### Windows (Visual Studio)
From the project root directory:
```bash
scripts\build_test.bat
```

### Manual Build
From the project root directory:
```bash
cd test
mkdir build
cd build
cmake .. -G "Visual Studio 16 2019" -A x64
cmake --build . --config Release
Release\xbus_parser_test.exe
```

### Linux/macOS
```bash
cd test
mkdir build
cd build
cmake ..
make
./xbus_parser_test
```

## Expected Output

When all tests pass, you should see:
```
=== XBus Parser Test Suite ===


--- Testing MTData2 with All Components ---
[PASS] MTData2 parsing success
[PASS] Has PacketCounter
[PASS] Has SampleTimeFine
[PASS] Has EulerAngles
[PASS] Has StatusWord
[PASS] Has LatLon
[PASS] Has AltitudeEllipsoid
[PASS] Has VelocityXYZ
[PASS] PacketCounter value
[PASS] SampleTimeFine value
[PASS] StatusWord value
[PASS] Euler Roll (expected: 179.933, actual: 179.933)
[PASS] Euler Pitch (expected: -1.15054, actual: -1.15054)
[PASS] Euler Yaw (expected: -2.342, actual: -2.342)
[PASS] Latitude (expected: 31.3932, actual: 31.3932)
[PASS] Longitude (expected: 121.23, actual: 121.23)
[PASS] Altitude (expected: 56.715, actual: 56.715)
[PASS] Velocity X (expected: -0.021543, actual: -0.021543)
[PASS] Velocity Y (expected: 0.0137628, actual: 0.0137628)
[PASS] Velocity Z (expected: -0.0434888, actual: -0.0434888)

--- Testing FP1632 Conversion ---
[PASS] Latitude FP1632 conversion (expected: 31.3932, actual: 31.3932)
[PASS] Longitude FP1632 conversion (expected: 121.23, actual: 121.23)
[PASS] Altitude FP1632 conversion (expected: 56.715, actual: 56.715)

--- Testing Euler Angles Only ---
[PASS] Euler only parsing success
[PASS] Has EulerAngles
[PASS] No LatLon
[PASS] No VelocityXYZ
[PASS] Euler Roll (45.0) (expected: 45, actual: 45)
[PASS] Euler Pitch (30.0) (expected: 30, actual: 30)
[PASS] Euler Yaw (90.0) (expected: 90, actual: 90)

--- Testing LatLon Only ---
[PASS] LatLon only parsing success
[PASS] Has LatLon
[PASS] No EulerAngles
[PASS] Latitude (1.0) (expected: 1, actual: 1)
[PASS] Longitude (-1.0) (expected: -1, actual: -1)

--- Testing Velocity Only ---
[PASS] Velocity only parsing success
[PASS] Has VelocityXYZ
[PASS] No EulerAngles
[PASS] Velocity X (0.1) (expected: 0.1, actual: 0.1)
[PASS] Velocity Y (0.2) (expected: 0.2, actual: 0.2)
[PASS] Velocity Z (0.3) (expected: 0.3, actual: 0.3)

--- Testing Invalid Message ---
[PASS] Invalid preamble rejection
[PASS] Wrong message ID rejection

=== Test Results ===
Passed: 43/43
All tests PASSED!
```

## Data Sources

The test data is extracted from real Xsens sensor output as shown in the MT Manager screenshots, ensuring the parser correctly handles actual sensor data formats.