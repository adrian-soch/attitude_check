#!/bin/bash

confirm() {
    read -p "Are you sure you want to proceed with installation? (yes/no): " response
    if [[ "${response,,}" == "yes" ]]; then
        return 0
    else
        echo "Installation aborted."
        return 1
    fi
}

confirm_unistall() {
    read -p "Are you sure you want to proceed with uninstallation? You may have other dependent programs. (yes/no): " response
    if [[ "${response,,}" == "yes" ]]; then
        return 0
    else
        echo "Installation aborted."
        return 1
    fi
}

if [ "$1" == "install" ]; then
    if confirm; then
        sudo apt install cmake lcov cppcheck
        sudo apt-get install libgtest-dev && cd /usr/src/gtest && sudo cmake CMakeLists.txt
        sudo apt-get install libeigen3-dev && sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
        echo "Done installing build dependencies."
    fi
elif [ "$1" == "develop" ]; then
    if confirm; then
        pip install pre-commit
        pre-commit install
        sudo apt install cmake lcov cppcheck
        sudo apt-get install libgtest-dev && cd /usr/src/gtest && sudo cmake CMakeLists.txt
        sudo apt-get install libeigen3-dev && sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
        echo "Done developer setup."
    fi
elif [ "$1" == "uninstall" ]; then
    if confirm_unistall; then
        sudo apt-get remove --purge cmake lcov cppcheck libgtest-dev libeigen3-dev
        sudo rm /usr/local/include/Eigen
        echo "Uninstall complete."
    fi
else
    echo "Invalid input. Please use 'install', 'develop', or 'uninstall'."
fi
