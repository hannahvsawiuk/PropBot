#! /bin/bash

set -e

image=${1:-propbot-ros-base}

script_dir=$(dirname "$0")

repo_dir=$(cd "$script_dir/.." && pwd)

# Run container interactively
run_args=(
    -i
    -t
    # Bind mount the repo into home
    "--volume=$repo_dir:/home/$USER/PropBot:rw"
)

if [ "$(uname)" = Darwin ]; then
    # MacOS specific display setup with X11 Quartz
    xhost + 127.0.0.1
    run_args+=(
    # Pass through the display
    "--env=DISPLAY=host.docker.internal:0"
    )
else
    # Generic Linux display setup
    run_args+=(
    "--env=DISPLAY"
    "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"
    )
fi

echo "Running image $image..."

docker run "${run_args[@]}" "$image"
