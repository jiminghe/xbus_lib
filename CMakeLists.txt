cmake_minimum_required(VERSION 3.10)

project(XbusSerialReader)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags for Windows
if(WIN32)
    add_definitions(-DWIN32_LEAN_AND_MEAN)
    add_definitions(-DNOMINMAX)
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/xbus)

# Xbus library source files
set(XBUS_SOURCES
    xbus/xbus.cpp
    xbus/xbus_parser.cpp
)

# Xbus library headers
set(XBUS_HEADERS
    xbus/xbus.h
    xbus/xbus_parser.h
    xbus/xbus_message_id.h
)

# Create Xbus static library
add_library(xbus STATIC ${XBUS_SOURCES} ${XBUS_HEADERS})

# Serial reader source files
set(SERIAL_SOURCES
    serial_reader.cpp
)

# Serial reader headers
set(SERIAL_HEADERS
    serial_reader.h
)

# Create serial reader static library
add_library(serial_reader STATIC ${SERIAL_SOURCES} ${SERIAL_HEADERS})

# Main executable
add_executable(xbus_reader main.cpp)

# Link libraries
target_link_libraries(xbus_reader xbus serial_reader)

# Link Windows specific libraries
if(WIN32)
    target_link_libraries(xbus_reader winmm)
endif()

# Set output directory
set_target_properties(xbus_reader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Copy executable to root directory for convenience
add_custom_command(TARGET xbus_reader POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:xbus_reader> ${CMAKE_CURRENT_SOURCE_DIR}/
)