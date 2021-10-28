#!/bin/sh

# the script check if the public repo and the local repo commit history are sync, otherwise it fail
cd src/franka_ros
public_url="https://github.com/frankaemika/franka_ros.git"
public_remote_name=$(git remote -v | grep ${public_url} | head -n 1 | sed -e 's/\s.*$//')
develop_branch_name="develop"

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
