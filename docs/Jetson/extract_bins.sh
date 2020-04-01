#! /bin/bash

set -e

script_dir=$(dirname "$0")

cd "$script_dir"

GCC_TAR=gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz
TEGRA_SOURCES_TAR=public_sources.tbz2
TEGRA_TOOLS_TAR=Tegra210_Linux_R32.3.1_aarch64.tbz2
ROOTFS_TAR=Tegra_Linux_Sample-Root-Filesystem_R32.3.1_aarch64.tbz2

required_bins=(
    $GCC_TAR
    $TEGRA_SOURCES_TAR
    $TEGRA_TOOLS_TAR
    $ROOTFS_TAR
)

check_files=(
    build/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc
    build/Linux_for_Tegra/flash.sh
    build/Linux_for_Tegra/rootfs/README.txt
    build/Linux_for_Tegra/source/public/kernel_src.tbz2
    build/Linux_for_Tegra/tools/jetson-disk-image-creator.sh
)

if [ -d "build" ]; then
    echo "build directory exists, delete it and rerun"
    exit 1
fi

for bin in "${required_bins[@]}"; do
    if [ ! -f "$bin" ]; then
        echo "Missing Binary: $bin"
        exit 1
    fi
done

mkdir build

for bin in "$GCC_TAR" "$TEGRA_TOOLS_TAR" "$TEGRA_SOURCES_TAR"; do
    echo "Extracting $bin..."
    tar -C "build" -xvf "$PWD/$bin"
    echo "Done"
done

for f in "${check_files[@]}"; do
    if [ ! -f "$f" ]; then
        echo "Expected file $f after extracting but it does not exist!"
        echo "Perhaps the tarball you downloaded is not the correct version?"
        exit 1
    fi
done

echo "Done extracting all bins!"
