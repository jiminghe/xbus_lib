# Test CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Include the xbus library headers from parent directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../xbus)

# Create test executable
add_executable(xbus_parser_test 
    test_xbus_parser.cpp
    ../xbus/xbus.cpp
    ../xbus/xbus_parser.cpp
)

# Set C++ standard
set_property(TARGET xbus_parser_test PROPERTY CXX_STANDARD 11)

# Add test target
enable_testing()
add_test(NAME parser_test COMMAND xbus_parser_test)