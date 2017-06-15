#!/bin/sh

set -e

. /opt/ros/kinetic/setup.sh

rm -rf build devel
catkin_make --cmake-args -DFranka_DIR:PATH=$FRANKA_DIR -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. devel/setup.sh
catkin_make -j1 check-format
catkin_make -j1 check-tidy
catkin_make run_tests
