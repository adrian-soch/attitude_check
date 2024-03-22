pip install pre-commit
pre-commit install
sudo apt install cmake lcov cppcheck
sudo apt-get install libgtest-dev && cd /usr/src/gtest && sudo cmake CMakeLists.txt
sudo apt-get install -y libeigen3-dev && sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen

echo "Done setup."
