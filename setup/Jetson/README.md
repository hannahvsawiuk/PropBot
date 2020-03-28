# Building Custom Rootfs for Jetson TX1

This will tell you how to build the Jetson TX1's kernel and root filesystem and
flash it and boot from a SSD.

Note that this is based on this [tutorial](https://developer.ridgerun.com/wiki/index.php?title=JetsonTX2/Getting_Started/Compiling_Jetson_TX2_source_code).
It is somewhat out of date but most of the steps are correct.

Another good resource is this [github repo](https://github.com/jetsonhacks/buildLibrealsense2TX).
Although it only works on a older version of librealsense and Jetpack.

## Downloading the Sources and Binaries

The various tarballs required can be downloaded from [Nvidia's Archive](https://developer.nvidia.com/embedded/linux-tegra-archive).
As of writing these instructions the latest version is Linux for Tegra 32.3.1,
this is the version we will use.

The following files need to be downloaded:

### GCC 7.3.1 for 64 bit BSP and Kernel

File: `gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz`

This contains the aarch64 cross compile toolchain used to compile the kernel
and userspace programs for the Jetson TX1.

### L4T Driver Package (BSP)

File: `Tegra210_Linux_R32.3.1_aarch64.tbz2`

This contains various tools for building the rootfs and flashing the TX1.

### L4T Driver Package (BSP) Sources

File: `public_sources.tbz2`

This contains the Linux kernel and other driver sources for the TX1.

### Sample Root Filesystem

File: `Tegra_Linux_Sample-Root-Filesystem_R32.3.1_aarch64.tbz2`

This is the actual root filesystem Nvidia uses for the TX1 despite being called
"Sample".


## Building

### Extracting Binaries

Run the extract script which will extra all the archives into their locations

```sh
$ ./extract_bins.sh
```

### Building the Kernel

Build the kernel, this also applies the patches for the realsense

```sh
$ ./build_kernel.sh
```

### Flashing TX1

There are two storage configurations possible:
- Internal Storage
- Sata SSD (Do NOT use a hard drive as they take too much power)

#### Internal Storage

To flash:

```sh
$ ./flash_tx1.sh
```

#### Sata Drive

A Sata drive can be connected to your computer using a Sata to USB adapter
which can be found cheaply online or at a computer parts store like memory express.

Plug the drive into your computer and note its device name.
Usually this is /dev/sda, /dev/sdb, ... depending on how many
disks your computer already has.

Drive Format:
- The drive should be using a GPT partition table
- The drive should exactly 1 partition named /dev/sdX1 (X = a,b,c,...)

If the drive isn't already formatted this way you can use gparted to format it.

Assuming that the drive is /dev/sdf then to flash:

```sh
$ ./flash_tx1.sh /dev/sdf1
```

After flashing unplug the drive and plug it into the TX1's Sata port.
The flash script will have restarted the TX1 automatically but it will have
failed booting due to the missing drive. Press the Reset button once to reboot the TX1.

### Post Flash

After flashing with one of the methods the setup must be completed.
To do this, connect the TX1 to a monitor, keyboard, and mouse. This setup process
is the same as setting up any other Ubuntu Linux system for the first time.

After you are done setup it is recommended to update the system.

Open a terminal on the TX1 and run:
```sh
$ sudo apt update
$ sudo apt upgrade
```

### Nvidia Drivers/Libraries

Another thing missing will be CUDA, OpenCV, and other libraries that Nvidia
installs on Jetsons. This can be installed via the Nvidia SDKManager. The program
is free but you must register as a Nvidia developer with your UBC alumni email.

After installing and opening SDKManager select the Jetson TX1 and the same version
of Jetpack you installed (Newer one may be backwards compatible).

At the second step make sure to disable the option for downloading of the Jetson TX1 OS
and flashing it. We only need SDKManager to install cuda and other stuff on the TX1.

Make sure the Jetson TX1 has a fast internet connection and that you can SSH into it from
the computer running SDKManager. Now proceed onto the next step, SDKManager will ask for
the IP address and login credentials for the TX1. Provide these and let the installation run.

The process overall easily takes 20+ minutes, even worse if on a slow internet connection.
After it is done the Jetson TX1 should have CUDA and other libraries onboard.
