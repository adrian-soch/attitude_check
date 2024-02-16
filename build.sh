#!/bin/bash
# Usage: ./build.sh [test|clean|debug]

# Create a build directory if it does not exist
mkdir -p build && cd build

# Check the first argument
case $1 in
  test)
    # Build the project with tests enabled
    cmake -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage" ..
    make
    cd ./test
    # Run the tests
    ctest

    # Generate the coverage report using gcov and lcov
    lcov --capture --directory ../src --output-file coverage.info
    lcov --remove coverage.info '/usr/*' --output-file coverage.info
    lcov --list coverage.info
    genhtml coverage.info --output-directory out
    ;;
  clean)
    # Remove the build directory
    cd .. && rm -rf build
    ;;
  debug)
    # Build the project with debug mode enabled
    cmake -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Debug ..
    make
    ;;
  *)
    # Build the project with tests disabled and release mode
    cmake -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release ..
    make
    ;;
esac
