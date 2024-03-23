set(BINARY ${CMAKE_PROJECT_NAME}_test)

# Assumes GoogleTest is already installed
find_package(GTest REQUIRED)

# The following commands pulls the newest version of gtest automoatically
# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/refs/heads/main.zip
# )
# # For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)

enable_testing()

set(CMAKE_CXX_FLAGS "-fprofile-arcs -ftest-coverage")

add_executable(
    ${BINARY}
    main.cpp
    attitude_check_test.cpp
    # attitude_check_estimation_test.cpp
    quaternion_test.cpp
    utilities_test.cpp
    initializers_test.cpp
)

target_link_libraries(
    ${BINARY}
    PUBLIC ${CMAKE_PROJECT_NAME}_lib
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(${BINARY})
