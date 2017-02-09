#!/bin/sh

set -e

. /opt/ros/kinetic/setup.sh

rm -rf build devel
catkin_make --cmake-args -DFranka_DIR:PATH=$FRANKA_DIR
. devel/setup.sh
catkin_make check-format
catkin_make check-tidy
catkin_make test
