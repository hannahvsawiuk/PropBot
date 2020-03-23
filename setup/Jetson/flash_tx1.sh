#! /bin/bash

set -e

# Flash the Jetson TX1

_build_dir=$PWD/build
_tegra_dir=$PWD/build/Linux_for_Tegra
_tegra_kernel_dir=$_tegra_dir/kernel
_image_dir=$_tegra_dir/images
_gcc_ver=gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu
_target=aarch64-linux-gnu
