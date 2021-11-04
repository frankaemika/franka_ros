#!/bin/sh
set -e

# the script checks if the public repo and the local repo commit history are sync, otherwise it fails
if [ $# -ne 2 ]; then
    >&2 echo "Not enough argument supplied. Usage: checkgithistory.sh public_url develop_branch_name"
    exit 1
fi

cd src/franka_ros
public_url=$1
public_remote_name=$(git remote -v | grep ${public_url} | head -n 1 | sed -e 's/\s.*$//')
develop_branch_name=$2

if [ -z "$public_remote_name" ]; then
    public_remote_name="public"
    git remote add $public_remote_name $public_url
fi

git fetch $public_remote_name

local_commit_hash=$(git rev-parse origin/${develop_branch_name})
public_commit_hash=$(git rev-parse ${public_remote_name}/${develop_branch_name})

if [ "$local_commit_hash" != "$public_commit_hash" ]; then
    >&2 echo "Local and public commit hashes do not match. Please update local to the latest public commit"
    exit 1
else
    echo "Local and public commit hashes do match."
fi
