#! /bin/bash

set -e

# Build the Jetson TX1 kernel
#
# This also applies patches to the V4L driver support the Intel Realsense
# See https://github.com/IntelRealSense/librealsense
#
# Note that the patches from the librealsense repo cannot be directly applied to
# the TX1's Linux 4.9 kernel so we ported the patches over manually. These
# are a combination of the following patch versions in librealsense:
# realsense-*-bionic-master.patch <-- mostly based on these patches
# realsense-*-xenial-Ubuntu-hwe-4.8.0-58.63_16.04.1.patch <-- only used some hid stuff from here

_build_dir=$PWD/build
_tegra_dir=$PWD/build/Linux_for_Tegra
_tegra_kernel_dir=$_tegra_dir/kernel
_image_dir=$_tegra_dir/images
_gcc_ver=gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu
_target=aarch64-linux-gnu
_cross_compile=$_build_dir/$_gcc_ver/bin/$_target-

_kernel_src_tar=$PWD/build/Linux_for_Tegra/source/public/kernel_src.tbz2
_kernel_build_dir=$PWD/build_kernel
_kernel_modules_out_dir=$_image_dir/modules
_kernel_out_dir=$_image_dir
_kernel_config=$_kernel_out_dir/.config
_kernel_version=4.9.140-tegra

# Our custom patches
_patch_camera_fmts=$PWD/realsense_patches/propbot-realsense-camera-formats.patch
_patch_metadata=$PWD/realsense_patches/propbot-realsense-metadata.patch
_patch_hid=$PWD/realsense_patches/propbot-realsense-hid.patch
_patch_powerline=$PWD/realsense_patches/propbot-realsense-powerlinefrequency-control-fix.patch

if [ ! -f "${_cross_compile}gcc" ]; then
    echo "Expected toolchain in $_build_dir/$_gcc_ver, but it was not found"
    echo "Build environment was not set up."
    exit 1
fi

if [ ! -d "$_tegra_dir" ]; then
    echo "Expected tegra tools in $_tegra_dir, but it was not found"
    echo "Build environment was not set up."
    exit 1
fi

if [ ! -f "$_kernel_src_tar" ]; then
    echo "Expected kernel source tar at $_kernel_src_tar, but it was not found"
    echo "Build environment was not set up."
    exit 1
fi

_source_dir="$_kernel_build_dir/kernel/kernel-4.9/"

# shellcheck disable=SC2191
_common_make_args=(
    -C "${_source_dir}"
    O="${_kernel_out_dir}" 
    LOCALVERSION="-tegra"
    ARCH=arm64
    CROSS_COMPILE=${_cross_compile}
    -j"$(nproc)"
    --output-sync=target
)

if [ ! -d "$_source_dir" ]; then
    rm -rf "$_kernel_build_dir"
    mkdir -p "$_kernel_build_dir"
    mkdir -p "$_image_dir"

    echo "Extracting Kernel Sources $_kernel_src_tar to $_kernel_build_dir"
    tar -C "$_kernel_build_dir" -xf "$_kernel_src_tar"
else
    echo "Kernel already extracted, cleaning build directory"
    echo "Remove $_kernel_build_dir if you and to restart from scratch"
fi

_apply_patch() {
    local patch=$1
    local name=$2

    echo "Applying $name patch"
    # Try applying in reverse to check if already applied
    if patch -s -f -R --dry-run -p1 < "$patch" &> /dev/null; then
        echo "Already Applied $name patch"
        return 0
    elif patch -p1 < "$patch"; then
        echo "Successfully Applied $name patch"
        return 0
    else
        echo "Could not apply $name patch!"
        return 1
    fi
}

_change_kernel_config() {
    local config="CONFIG_$1"
    local config_val="$2"
    local desc="$config to ${config}=${config_val}"

    echo "Changing kernel config: $desc"
    if grep -q -e "^${config}=${config_val}" "$_kernel_config"; then
        echo "Already changed kernel config: $desc"
        return 0
    else
        # Note that a space is used to prevent match vars with same prefixes
        sed -i -e "s/.*${config} .*/${config}=${config_val}/" "$_kernel_config"

        if grep -q -e "^${config}=${config_val}" "$_kernel_config"; then
            echo "Successfully changed kernel config: $desc"
        else
            echo "Failed to changed kernel config: $desc"
            return 1
        fi
    fi
}

pushd "$_source_dir"
_apply_patch "$_patch_camera_fmts" "Realsense camera formats"
_apply_patch "$_patch_metadata" "Realsense metadata"
_apply_patch "$_patch_hid" "Realsense HID"
_apply_patch "$_patch_powerline" "Realsense powerline frequency"
popd

echo "Making: mrproper"
make "${_common_make_args[@]}" mrproper

echo "Making: tegra_defconfig"
make "${_common_make_args[@]}" tegra_defconfig

_change_kernel_config IIO y
_change_kernel_config IIO_BUFFER y
_change_kernel_config IIO_KFIFO_BUF y
_change_kernel_config IIO_TRIGGERED_BUFFER y
_change_kernel_config IIO_TRIGGER y
_change_kernel_config IIO_CONSUMERS_PER_TRIGGER 2
_change_kernel_config HID_SENSOR_IIO_COMMON m
_change_kernel_config HID_SENSOR_HUB m
_change_kernel_config HID_SENSOR_ACCEL_3D m
_change_kernel_config HID_SENSOR_GYRO_3D m

echo "Making: zImage"
make "${_common_make_args[@]}" zImage

echo "Making: dtbs"
make "${_common_make_args[@]}" dtbs

echo "Making: modules"
make "${_common_make_args[@]}" modules

echo "Making: modules_install"
make "${_common_make_args[@]}" modules_install \
    INSTALL_MOD_PATH="${_kernel_modules_out_dir}"

# Packaging

# Create the tarball of the kernel modules
pushd "$_kernel_out_dir/modules"
# delete symlinks
rm -f lib/modules/$_kernel_version/source \
    lib/modules/$_kernel_version/build

echo "Tarring Kernel Modules"
tar -cjf kernel_supplements.tbz2 lib
mv kernel_supplements.tbz2 "$_tegra_kernel_dir"
popd

# Copy the device trees
pushd "$_kernel_out_dir/arch/arm64/boot"
echo "Copying DTBs"
cp -v dts/*.dtb "$_tegra_kernel_dir/dtb"
echo "Copying Kernel Images"
cp -v Image zImage "$_tegra_kernel_dir"
popd
