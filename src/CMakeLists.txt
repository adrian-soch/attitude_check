set(BINARY ${CMAKE_PROJECT_NAME})

add_executable(${BINARY} main.cpp attitude_check.cpp attitude_check.h)
add_library(${BINARY}_lib STATIC main.cpp attitude_check.cpp attitude_check.h)
