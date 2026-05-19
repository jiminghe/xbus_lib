cmake_minimum_required(VERSION 3.16)

project(XbusTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../xbus)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../xsmath)

add_executable(xbus_parser_test
    test_xbus_parser.c
    ../xbus/xbus.c
    ../xbus/xbus_parser.c
)
target_link_libraries(xbus_parser_test m)

add_executable(xsmath_test
    test_xsmath.c
    ../xsmath/xsquaternion.c
    ../xsmath/xseuler.c
)
target_link_libraries(xsmath_test m)

enable_testing()
add_test(NAME parser_test COMMAND xbus_parser_test)
add_test(NAME xsmath_test COMMAND xsmath_test)
