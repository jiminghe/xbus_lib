cmake_minimum_required(VERSION 3.10)

project(XbusSerialReader C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/xbus)

set(XBUS_SOURCES
    xbus/xbus.c
    xbus/xbus_parser.c
)

set(XBUS_HEADERS
    xbus/xbus.h
    xbus/xbus_parser.h
    xbus/xbus_message_id.h
)

add_library(xbus STATIC ${XBUS_SOURCES} ${XBUS_HEADERS})

set(SERIAL_SOURCES serial_reader.c)
set(SERIAL_HEADERS serial_reader.h)

add_library(serial_reader STATIC ${SERIAL_SOURCES} ${SERIAL_HEADERS})

add_library(xsmath STATIC
    xsmath/xsquaternion.c
    xsmath/xseuler.c
    xsmath/xsquaternion.h
    xsmath/xseuler.h
)
target_include_directories(xsmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/xsmath)
target_link_libraries(xsmath PUBLIC m)

add_executable(xbus_reader main.c xsdevice_def.c)
target_link_libraries(xbus_reader xbus serial_reader)

set_target_properties(xbus_reader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
