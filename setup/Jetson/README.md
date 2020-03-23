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

