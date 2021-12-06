#!/bin/sh

set -e
set -x

# check if build-debug folder exists and change directory
if [ ! -d build-debug ]; then
    2&> echo "build-debug folder does not exists"
    exit 1
fi
cd build-debug

# check if devel folder exists and source setup.sh
if [ ! -d devel ]; then
    2&> echo "devel folder does not exists"
    exit 1
fi
. devel/setup.sh

# testing
ctest -V
