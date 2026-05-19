cmake_minimum_required(VERSION 3.10)

project(XbusSerialReader CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/xbus)

set(XBUS_SOURCES
    xbus/xbus.cpp
    xbus/xbus_parser.cpp
)

set(XBUS_HEADERS
    xbus/xbus.h
    xbus/xbus_parser.h
    xbus/xbus_message_id.h
)

add_library(xbus STATIC ${XBUS_SOURCES} ${XBUS_HEADERS})

set(SERIAL_SOURCES serial_reader.cpp)
set(SERIAL_HEADERS serial_reader.h)

add_library(serial_reader STATIC ${SERIAL_SOURCES} ${SERIAL_HEADERS})

# Async reader uses std::thread.
find_package(Threads REQUIRED)
target_link_libraries(serial_reader PUBLIC Threads::Threads)

add_executable(xbus_reader main.cpp)
target_link_libraries(xbus_reader xbus serial_reader)

set_target_properties(xbus_reader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
