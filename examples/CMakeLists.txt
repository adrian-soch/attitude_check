set(BINARY ${CMAKE_PROJECT_NAME}_demo)

add_executable(${BINARY} basic_orientation_calculation.cpp)

target_link_libraries(
    ${BINARY}
    PUBLIC ${CMAKE_PROJECT_NAME}_lib
)
