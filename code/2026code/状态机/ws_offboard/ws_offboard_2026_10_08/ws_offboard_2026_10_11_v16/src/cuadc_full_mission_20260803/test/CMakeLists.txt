cmake_minimum_required(VERSION 3.8)

# Also supports a standalone build with only a C++17 compiler and CMake.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(cuadc_release_gate_tests LANGUAGES CXX)
  include(CTest)
endif()

add_executable(test_release_gate test_release_gate.cpp)
target_compile_features(test_release_gate PRIVATE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(test_release_gate PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

if(BUILD_TESTING)
  add_test(NAME release_gate COMMAND test_release_gate)
  set_tests_properties(release_gate PROPERTIES
    TIMEOUT 10
    LABELS "offline;regression;standalone")
endif()
