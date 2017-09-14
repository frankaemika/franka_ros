#!/bin/sh

set -e
set -x

. /opt/ros/kinetic/setup.sh

rm -f src/CMakeLists.txt
catkin_init_workspace src

rm -rf build-debug
mkdir build-debug
cd build-debug

cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ../src
. devel/setup.sh

cmake --build .
cmake --build . --target tests
ctest -V

cmake --build . --target check-format
cmake --build . --target check-tidy
