# Attitude Check: An IMU-based Attitude Estimator

---

[![C/C++ CI](https://github.com/adrian-soch/attitude_check/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/adrian-soch/attitude_check/actions/workflows/ci.yaml)
[![codecov](https://codecov.io/gh/adrian-soch/attitude_check/graph/badge.svg?token=2VGQ9KA5G8)](https://codecov.io/gh/adrian-soch/attitude_check)

<p align="center">
    <img src="https://github.com/adrian-soch/attitude_check/assets/6884645/aa21f86e-6b74-4a9d-a757-ec63950c2f18" alt="Gif of Attitude Check in action." />
</p>

This project implements the Madwick attitude estimation algorithm. The focus of this repository is high quality documentation.
The repo structure is:

```
    .
    ├── docs        # Documentation for the project
    ├── examples    # Example usage code for PC and Arduino
    ├── scripts     # Scripts for setup and supporting work
    ├── src         # Source code
    └── test        # Test cases

```

---

## Install

See [INSTALL.md](./INSTALL.md) for details on how to install and get started with this project on your own computer.

## Usage

Use the provided `./build.sh` to build a release version of attitude check.

To clean the build output run `./build.sh clean`.

### Unit Tests

To build and run unit tests, including code coverage reports run `./build.sh test`.

### Arduino

The Arduino examples are written for the SparkFun Razor 9dof IMU. However, the usage should be
clear for other boards. See the `#include`s for the libraries you will need to install.

## Contributing

See [CONTRIBUTING.md](./CONTRIBUTING.md) for details on how to contribute to this project.

## Troubleshooting and Comments

This section will be updated to inform potential users of known issues and common solutions.

### Known Issues
- Arduino doesn't like Real C++ code, other boards may not like using the ArduinoEigen library.
