# Xbus Serial Reader

A C++ implementation of the Xbus protocol for communicating with Xsens motion tracking devices via serial communication on Windows.

## Overview

This project provides a complete C++ library and application for reading and parsing Xbus messages from Xsens motion tracking devices. It includes a Windows serial port interface and real-time message processing with support for extracting Euler angles (roll, pitch, yaw) from XMID_MtData2 messages.

## Features

- **Modern C++ Implementation**: Converted from C to modern C++17 with proper class structure
- **Windows Serial Communication**: Native Windows API implementation for robust COM port handling
- **Asynchronous Message Processing**: Real-time message reception with automatic synchronization
- **Comprehensive Message Support**: Support for various Xbus message types including:
  - Device identification
  - Firmware revision
  - Configuration mode switching
  - Real-time measurement data (Euler angles)
- **Interactive Commands**: Runtime commands for device control
- **Robust Error Handling**: Proper checksum verification and error reporting
- **Easy Build System**: CMake-based build with convenient batch scripts

## Project Structure

```
├── scripts/                  # Build automation scripts
│   ├── build.bat            # Build the project
│   └── clean.bat            # Clean build files
├── xbus/                    # Xbus protocol library
│   ├── xbus.hpp             # Main Xbus class
│   ├── xbus.cpp             # Xbus implementation
│   ├── xbus_message_id.hpp  # Message ID definitions
│   ├── xbus_parser.hpp      # Message parsing utilities
│   └── xbus_parser.cpp      # Parser implementation
├── serial_reader.h          # Windows serial port interface
├── serial_reader.cpp        # Serial port implementation
├── main.cpp                 # Main application
├── CMakeLists.txt          # CMake build configuration
└── README.md               # This file
```

## Requirements

### Hardware
- Xsens motion tracking device (MTi series)
- USB-to-Serial adapter or direct serial connection
- Windows PC with available COM port

### Software
- Windows 10/11
- Visual Studio 2019 or later (with C++ tools)
- CMake 3.10 or later
- Git (optional, for cloning)

## Quick Start

### 1. Clone or Download
```bash
git clone <repository-url>
cd xbus_lib
```

### 2. Build the Project
Run the build script:
```cmd
scripts\build.bat
```

Or build manually:
```cmd
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### 3. Connect Your Device
- Connect your Xsens device to a COM port (e.g., COM4)
- Note the COM port number
- Ensure the device is powered on

### 4. Run the Application
```cmd
build\bin\Release\xbus_reader.exe
```

## Configuration

### Changing COM Port
Edit `main.cpp` and modify this line:
```cpp
if (!processor.initialize("COM4")) {
```
Change `"COM4"` to your desired port (e.g., `"COM3"`, `"COM5"`).

### Serial Settings
Default settings are:
- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1

These can be modified in the `SerialReader::open()` call in `main.cpp`.

## Usage

### Interactive Commands
Once the application is running, you can use these commands:

| Command | Action |
|---------|--------|
| `i` | Request device information |
| `c` | Switch to configuration mode |
| `m` | Switch to measurement mode |
| `f` | Request firmware revision |
| `q` | Quit application |

### Example Output
```
Xbus Serial Reader
==================
Serial port COM4 opened successfully at 115200 baud.
Started listening for Xbus messages...
Press 'q' and Enter to quit, 'i' for device info, 'c' to go to config mode, 'm' to go to measurement mode.
Received: XMID_MtData2: roll = 1.23, pitch = -0.45, yaw = 89.67
  -> Roll: 1.23°, Pitch: -0.45°, Yaw: 89.67°
```

## API Reference

### Xbus Class
Main static class for Xbus protocol operations:

```cpp
// Message validation
static bool checkPreamble(const uint8_t* message);
static bool verifyChecksum(const uint8_t* message);

// Message creation and parsing
static void createMessage(uint8_t* buffer, uint8_t bid, uint8_t mid, uint16_t len);
static int getMessageId(const uint8_t* message);
static int getPayloadLength(const uint8_t* message);

// Raw message handling
static size_t createRawMessage(uint8_t* dest, const uint8_t* message);
```

### XbusParser Class
High-level message parsing utilities:

```cpp
// Parse specific message types
static std::string messageToString(const uint8_t* message);
static bool parseEulerAngles(const uint8_t* message, EulerAngles& angles);
static uint32_t parseDeviceId(const uint8_t* message);
```

### SerialReader Class
Windows serial port communication:

```cpp
// Basic operations
bool open(const std::string& portName, DWORD baudRate = 115200);
void close();
bool write(const uint8_t* data, size_t length);
int read(uint8_t* buffer, size_t bufferSize, DWORD timeoutMs = 1000);

// Asynchronous reading
void setDataCallback(std::function<void(const uint8_t*, size_t)> callback);
bool startAsyncReading();
void stopAsyncReading();
```

## Supported Message Types

| Message ID | Name | Description |
|------------|------|-------------|
| 0x3E | XMID_Wakeup | Wake up device |
| 0x3F | XMID_WakeupAck | Wake up acknowledgment |
| 0x00 | XMID_ReqDid | Request device ID |
| 0x01 | XMID_DeviceId | Device ID response |
| 0x30 | XMID_GotoConfig | Switch to configuration mode |
| 0x31 | XMID_GotoConfigAck | Configuration mode acknowledgment |
| 0x10 | XMID_GotoMeasurement | Switch to measurement mode |
| 0x11 | XMID_GotoMeasurementAck | Measurement mode acknowledgment |
| 0x36 | XMID_MtData2 | Motion data (Euler angles) |
| 0x12 | XMID_ReqFirmwareRevision | Request firmware version |
| 0x13 | XMID_FirmwareRevision | Firmware version response |

## Troubleshooting

### Common Issues

**COM Port Access Denied**
- Ensure no other application is using the COM port
- Check that the device drivers are properly installed
- Try running as administrator

**No Messages Received**
- Verify the correct COM port is being used
- Check that the device is powered on and connected
- Ensure the baud rate matches the device configuration (115200)

**Build Errors**
- Ensure Visual Studio C++ tools are installed
- Make sure CMake is in your system PATH
- Try cleaning and rebuilding: `scripts\clean.bat` then `scripts\build.bat`

**Checksum Errors**
- Check for loose connections
- Verify the serial port settings match the device
- Ensure no electrical interference

### Debug Mode
To enable more verbose output, modify the `main.cpp` file and add debug prints in the message processing functions.

## Build Scripts

### build.bat
Automates the complete build process:
- Creates build directory
- Runs CMake configuration
- Builds the Release version
- Reports success/failure

### clean.bat
Cleans all build artifacts:
- Removes build directory
- Asks for confirmation before deletion
- Removes copied executables

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly with actual hardware
5. Submit a pull request

## License

This project is provided as-is for educational and development purposes. Please check with Xsens for any licensing requirements related to their protocols.

## Acknowledgments

- Based on the original Xbus protocol specification from Xsens
- Converted from C implementation to modern C++
- Windows serial communication using native Win32 API