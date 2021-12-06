#!/bin/sh
# the script checks if the public repo and the local repo commit history are sync, otherwise it fails
set -e

PUBLIC_URL=$1
BRANCH=${2:-master}   # if no second arg supplied, then "master" is used

if [ $# -lt 1 ]; then
    >&2 echo "Not enough argument supplied. Usage: checkgithistory.sh PUBLIC_URL [BRANCH]"
    exit 1
fi

if [ ! -d ".git/" ] ; then
    echo "Current directory `pwd` does not appear to be a Git repository. Change directory before calling this script"
    exit 1
fi

public_remote_name=$(git remote -v | grep ${PUBLIC_URL} | head -n 1 | sed -e 's/\s.*$//')

if [ -z "$public_remote_name" ]; then
    public_remote_name="public"
    git remote add $public_remote_name $PUBLIC_URL
fi

git fetch $public_remote_name

local_commit_hash=$(git rev-parse origin/${BRANCH})
public_commit_hash=$(git rev-parse ${public_remote_name}/${BRANCH})

if [ "$local_commit_hash" != "$public_commit_hash" ]; then
    >&2 echo "Local and public commit hashes do not match. Please update local to the latest public commit"
    exit 1
else
    echo "Local and public commit hashes do match."
fi