#!/bin/bash

# Build an Optimized Google Cartographer for ROS
#
# This script sets up an catkin workspace to build cartographer
# and some of its immediate dependencies. The main goal is to optimize
# cartographer for the local machine.
#
# To use this you should have a cartographer workspace and a propbot
# workspace. The propbot workspace should extend the cartographer space.
#
# # Build Cartographer
# ./setup_cartographer_build.sh cartographer_ws
# cd cartographer_ws
# catkin build
# cd ..
#
# # Build Propbot
# cd propbot_ws
# catkin config --extend ../cartographer_ws/devel
# catkin build
# cd ..

set -e

if [ $# -lt 1 ]; then
    echo "Usage: $0 <workspace dir> [<extra cmake args...>]"
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "Not in ROS environment, source the distro's setup.bash"
    exit 1
fi

_check_for_command() {
    local cmd=$1
    local die=${2:-0}

    echo "Checking for command: $cmd"
    if command -v "$cmd"; then
        return 0
    else
        if [ "$die" -eq 1 ]; then
            echo "Command is required: $cmd"
            exit 1
        else
            echo "Command not found: $cmd"
            return 1
        fi
    fi
}

_check_run() {
    echo "Running Command: $*"

    if ! "$@"; then
        echo "Command failed: $*"
        exit 1
    fi
}

_download_file() {
    local url=$1
    local file=$2

    if [ -f "$file" ]; then
        echo "$file exists, not downloading"
        return 0
    fi

    echo "Downloading file: $file from $url"
    _check_run wget -O "$file" "$url"

    if [ ! -f "$file" ]; then
        echo "$file not downloaded"
        exit 1
    fi
}

_untar_into_dir() {
    local tar=$1
    local dir=$2

    shift 2

    if [ -d "$dir" ]; then
        echo "$dir exists, not untarring"
        return 0
    fi

    _check_run mkdir -p "$dir"

    _check_run tar -C "$dir" -xf "$tar" "$@"
}

_clone_repo() {
    local url=$1
    local dir=$2
    local revision=$3

    if [ ! -d "$dir/.git" ]; then
        echo "Cloning $url to $dir"
        rm -rf "$dir"
        _check_run git clone "$url" "$dir"
    fi

    pushd "$dir" > /dev/null
    echo "Fetching latest changes..."
    _check_run git fetch

    echo "Checking out $revision"
    _check_run git checkout "$revision"
    popd > /dev/null
}

echo "Finding script directory..."
pushd "$(dirname "$0")" > /dev/null
SCRIPT_DIR=$(pwd)
popd > /dev/null

_check_for_command wget 1
_check_for_command git 1
_check_for_command cmake 1
_check_for_command rosdep 1
_check_for_command catkin 1

WORKSPACE_ROOT=$1

shift

if [ ! -d "$WORKSPACE_ROOT" ]; then
    echo "Creating workspace: $WORKSPACE_ROOT"
    _check_run mkdir -p "$WORKSPACE_ROOT"
else
    echo "Workspace exists: $WORKSPACE_ROOT"
fi

_check_run cd "$WORKSPACE_ROOT"

if [ ! -d src ]; then
    _check_run mkdir src
fi

# ==== Eigen ====
EIGEN_VERSION=eigen-3.3.7.tar.gz
EIGEN_URL=https://gitlab.com/libeigen/eigen/-/archive/3.3.7/$EIGEN_VERSION
EIGEN_DIR=src/eigen

echo "Downloading Eigen..."
_download_file "$EIGEN_URL" "$EIGEN_VERSION"
_untar_into_dir "$EIGEN_VERSION" "$EIGEN_DIR" --strip-components=1
_check_run cp "$SCRIPT_DIR/eigen.xml" "$EIGEN_DIR/package.xml"

# ==== Ceres Solver ====
CERES_VERSION=ceres-solver-1.14.0.tar.gz
CERES_URL=http://ceres-solver.org/$CERES_VERSION
CERES_DIR=src/ceres-solver

echo "Downloading Ceres Solver..."
_download_file "$CERES_URL" "$CERES_VERSION"
_untar_into_dir "$CERES_VERSION" "$CERES_DIR" --strip-components=1
_check_run cp "$SCRIPT_DIR/ceres-solver.xml" "$CERES_DIR/package.xml"

# ==== Google Cartographer ====
#CARTOGRAPHER_VERSION=bcd5486025df4f601c3977c44a5e00e9c80b4975
CARTOGRAPHER_VERSION=master
CARTOGRAPHER_URL=https://github.com/Conhokis/cartographer
CARTOGRAPHER_DIR=src/cartographer

echo "Downloading Cartographer..."
_clone_repo "$CARTOGRAPHER_URL" "$CARTOGRAPHER_DIR" "$CARTOGRAPHER_VERSION"

# ==== Google Cartographer ROS ====
#CARTOGRAPHER_ROS_VERSION=1de03b3d32b9e4e5bc86aa9bfca948e592efd10d
CARTOGRAPHER_ROS_VERSION=master
CARTOGRAPHER_ROS_URL=https://github.com/googlecartographer/cartographer_ros
CARTOGRAPHER_ROS_DIR=src/cartographer_ros

echo "Downloading Cartographer ROS..."
_clone_repo "$CARTOGRAPHER_ROS_URL" "$CARTOGRAPHER_ROS_DIR" "$CARTOGRAPHER_ROS_VERSION"

echo "Initializing Workspace"
_check_run catkin init

# shellcheck disable=SC2191
cmake_args=(
    # Enable native platform detection, mainly so Eigen can detect AVX (x86) or NEON (arm)
    #-DCMAKE_C_FLAGS=-march=native
    #-DCMAKE_CXX_FLAGS=-march=native
    # Use -O3 and NDEBUG (CMake's default) or use package maintainer's recommended flags
    -DCMAKE_BUILD_TYPE=Release

    # Disable ceres solver's tests and examples
    -DBUILD_TESTING=OFF
    -DBUILD_EXAMPLES=OFF
    "$@"
)

# Use ccache when available since the build is freaking long
if _check_for_command ccache; then
    echo "Using ccache to accelerate rebuilds"

    # shellcheck disable=SC2191
    cmake_args+=(
        -DCMAKE_C_COMPILER_LAUNCHER=ccache
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    )
fi

# For whatever reason rosdep doesn't want to work with these packages
extra_apt_packages=(
    libatlas-base-dev
    libsuitesparse-dev
)

if ! dpkg -l "${extra_apt_packages[@]}" > /dev/null; then
    _check_run sudo apt-get install "${extra_apt_packages[@]}"
fi

_check_run rosdep install --os=ubuntu:bionic --from-paths src --ignore-src

_check_run catkin config --cmake-args "${cmake_args[@]}"

echo "Everything is done. To build, run: catkin build in $WORKSPACE_ROOT"
