#! /bin/bash

set -e

script_dir=$(dirname "$0")

cd "$script_dir"

echo "Building propbot-ros-base..."

build_args=(
    --build-arg "USERNAME=$USER"
)

if uid=$(id -u) && gid=$(id -g); then
    build_args+=(
    --build-arg "UID=$uid"
    --build-arg "GID=$gid"
    )
fi

docker build "${build_args[@]}" \
    -t propbot-ros-base:latest \
    build
