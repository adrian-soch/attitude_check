#!/bin/bash
# Usage: ./build.sh [test|clean|debug]

# Create a build directory if it does not exist
mkdir -p build && cd build

# Check the first argument
case $1 in
  test)
    # Build the project with tests enabled
    cmake -DBUILD_TESTS=ON ..
    make
    cd ./test
    # Run the tests
    ctest --rerun-failed --output-on-failure

    # Generate the coverage report using gcov and lcov
    cd ./CMakeFiles/attitude_check_test.dir

    lcov --capture --rc lcov_branch_coverage=1 --directory . --output-file coverage.info
    lcov --remove coverage.info '/usr/*' '*/test*' --output-file coverage.info
    lcov --list coverage.info
    genhtml coverage.info --output-directory ../../out
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
