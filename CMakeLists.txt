cmake_minimum_required(VERSION 3.14)

option(BUILD_TESTS "Build tests" OFF)

project(attitude_check)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# Add cppCheck for static checks
# set(CMAKE_CXX_CPPCHECK "cppcheck")
find_program(CMAKE_CXX_CPPCHECK NAMES cppcheck)
if (CMAKE_CXX_CPPCHECK)
    list(
        APPEND CMAKE_CXX_CPPCHECK
            "--enable=warning"
            "--inconclusive"
            "--force"
            "--inline-suppr"
            "--suppress=constStatement:${CMAKE_SOURCE_DIR}/src/attitude_check.cpp"
    )
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wpedantic -Werror)
endif()

include_directories(src examples)

add_subdirectory(src)
add_subdirectory(examples)

if(BUILD_TESTS)
  add_subdirectory(test)
endif()
