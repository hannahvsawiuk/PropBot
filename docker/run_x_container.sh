#! /bin/bash

set -e

image=${1:-propbot-ros-base}

# Run container interactively
run_args=(
    -i -t
)

if [ "$(uname)" = Darwin ]; then
    # MacOS Specific arguments
    xhost + 127.0.0.1
    run_args+=(
    "--env=DISPLAY=host.docker.internal:0"
    )
else
    # Generic Linux arguments
    run_args+=(
    "--env=DISPLAY"
    #"--volume=/etc/group:/etc/group:ro"
    #"--volume=/etc/passwd:/etc/passwd:ro"
    #"--volume=/etc/shadow:/etc/shadow:ro"
    #"--volume=/etc/sudoers.d:/etc/sudoers.d:ro"
    "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"
    )
fi

echo "Running image $image..."

docker run "${run_args[@]}" "$image"

