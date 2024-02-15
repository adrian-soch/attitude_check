cmake_minimum_required(VERSION 3.14)

option(BUILD_TESTS "Build tests" OFF)

project(attitude_check)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(src)

add_subdirectory(src)

if(BUILD_TESTS)
  add_subdirectory(test)
endif()