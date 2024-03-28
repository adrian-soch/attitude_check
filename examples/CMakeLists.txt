set(BINARY ${CMAKE_PROJECT_NAME}_demo)

add_executable(${BINARY} basic_orientation_calculation.cpp)

target_link_libraries(
    ${BINARY}
    PUBLIC ${CMAKE_PROJECT_NAME}_lib
)


# # Specify the path to your precompiled library
# set(LIB_DIR ${CMAKE_SOURCE_DIR}/build)

# # Add the precompiled library
# add_library(MyLibrary STATIC IMPORTED)
# set_target_properties(MyLibrary PROPERTIES IMPORTED_LOCATION ${LIB_DIR}/libattitude_check_lib.a)

# # Create the executable for main.cpp
# add_executable(${BINARY} main.cpp)

# # Link your precompiled library with the executable
# target_link_libraries(MyApp PRIVATE MyLibrary)
