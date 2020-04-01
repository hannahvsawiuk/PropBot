#! /bin/bash

set -e

# Flash the Jetson TX1

_tegra_dir=$PWD/build/Linux_for_Tegra
_storage=${1:-internal}
_is_internal=1
_rootfs_tar=$PWD/Tegra_Linux_Sample-Root-Filesystem_R32.3.1_aarch64.tbz2

case "$_storage" in
    internal)
        read -r -p "Flashing to Internal Storage. [enter] to continue"
        ;;
    *)
        if [ -b "$_storage" ]; then
            read -r -p "Flashing to device $_storage. [enter] to continue"
            _is_internal=0
        else
            echo "$_storage is not a valid storage device"
            exit 1
        fi
        ;;
esac

if [ ! -f "$_rootfs_tar" ]; then
    echo "Missing $_rootfs_tar"
    exit 1
fi

_check_for_tx1() {
    if lsusb -d 0955: | grep -i nvidia; then
        return 0
    fi

    return 1
}

_print_tx1_usb_instructions() {
cat <<EOF

Connecting the Jetson TX1:

Buttons

RST ??? REC PWR
+-+ +-+ +-+ +-+
|O| |O| |O| |O| < Power
+-+ +-+ +-+ +-+
 ^       ^
 Reset   Recovery

1. Plug a Micro USB cable from your computer into the Jetson TX1's Micro USB port.
2. Turn on the TX1 using the Power Button
3. While holding Recovery press the Reset button
4. Done

EOF
}

if ! _check_for_tx1; then
    _print_tx1_usb_instructions

    while ! _check_for_tx1; do
        read -r -p "Waiting for TX1... [enter] to continue"
    done
fi

echo "Found TX1"

if [ ! -f "$_tegra_dir/flash.sh" ]; then
    echo "Missing $_tegra_dir or flash.sh"
    echo "Have you run the other scripts?"
    exit 1
fi

cd "$_tegra_dir"

if [ $_is_internal = 1 ]; then
    echo "Removing existing rootfs..."
    sudo rm -rf rootfs
    mkdir rootfs

    _flash_dev=mmcblk0p1
else
    if grep -q "$_storage" /proc/mounts; then
        echo "Unmounting $_storage"
        sudo umount "$_storage"
    fi

    echo "Formatting $_storage as ext4"
    sudo mkfs.ext4 "$_storage"

    mkdir -p rootfs

    echo "Mounting $_storage"
    sudo mount "$_storage" rootfs

    # The TX1 always has the SATA port on sda
    _flash_dev=sda1
fi

echo "Extracting $_rootfs_tar..."
# Yes this must be extracted with sudo to get correct permissions
sudo tar -C rootfs -xf "$_rootfs_tar"

echo "Copying Binaries..."
sudo ./apply_binaries.sh -r "$PWD/rootfs"

echo "Syncing"
sync

sudo ./flash.sh jetson-tx1 $_flash_dev

if [ $_is_internal = 0 ]; then
    echo "Unmounting $_storage"
    sudo umount "$_storage"
fi

cat<<EOF
Done Flashing.

Plug the TX1 into a monitor with keyboard and mouse to finish setup.

EOF
