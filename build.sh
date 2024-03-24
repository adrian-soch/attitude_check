#!/bin/bash
# Usage: ./build.sh [test|clean|debug]

ROOT="$PWD"
BUILD="$ROOT/build"

# Create a build directory if it does not exist
mkdir -p $BUILD && cd $BUILD

# Check the first argument
case $1 in
  test)
    # Build the project with tests enabled
    cmake -DBUILD_TESTS=ON ..
    make || { echo "Build failed, stopping script."; exit 1; }
    cd "$BUILD/test"
    # Run the tests
    ctest --rerun-failed --output-on-failure

    # Generate the coverage report using gcov and lcov
    cd "$BUILD/test/CMakeFiles/attitude_check_test.dir"

    lcov --capture --rc lcov_branch_coverage=1 --directory . --output-file coverage.info
    lcov --remove coverage.info '/usr/*' '*/test*' --output-file coverage.info
    lcov --list coverage.info
    genhtml coverage.info --output-directory $BUILD/test/lcov_out
    ;;
  clean)
    # Remove the build directory
    cd "$ROOT" && rm -rf build
    ;;
  debug)
    # Build the project with debug mode enabled
    cmake -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
    make
    ;;
  *)
    # Build the project with tests disabled and release mode
    cmake -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release ..
    make
    ;;
esac
