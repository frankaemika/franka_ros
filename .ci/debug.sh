#!/bin/sh

set -e
set -x

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
cmake --build . --target check-pyformat
cmake --build . --target check-tidy

# check if the public repo and the local repo commit history are sync, otherwise it fail
cd ../src/franka_ros
public_url="https://github.com/frankaemika/franka_ros.git"
public_remote_name=$(git remote -v | grep ${public_url} | head -n 1 | sed -e 's/\s.*$//')
develop_branch_name="develop"

if [ -z "$public_remote_name" ]; then
    public_remote_name="public"
    git remote add $public_remote_name $public_url
fi

git fetch $public_remote_name

local_commit_hash=$(git log --all --oneline --decorate --no-abbrev-commit | grep origin/${develop_branch_name} | head -n 1 | sed -e 's/\s.*$//')
public_commit_hash=$(git log --all --oneline --decorate  --no-abbrev-commit | grep ${public_remote_name}/${develop_branch_name} | head -n 1 | sed -e 's/\s.*$//')

if [ "$local_commit_hash" != "$public_commit_hash" ]; then
    >&2 echo "Local and public commit hashes do not match. Please update local to the latest public commit"
    exit 1
fi
