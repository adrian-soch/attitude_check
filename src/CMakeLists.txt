set(BINARY ${CMAKE_PROJECT_NAME})

# add_executable(${BINARY} attitude_check.cpp attitude_check.hpp quaternion.hpp)
add_library(${BINARY}_lib STATIC attitude_check.cpp attitude_check.hpp quaternion.hpp)
