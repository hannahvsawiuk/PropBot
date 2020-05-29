# Jetson TX1

**These instructions are outdated but kept for reference.
Prefer the instructions in the Jetson directory**

These are instructions for re-flashing the OS on the Jetson TX1.

## Required Stuff

A Micro USB Cable is needed to do the flashing.

Additionally a network of some sort is needed. This can either be
an ethernet cable or Wifi without AP isolation. You can't use
public Wifi such as ubcsecure or ShawOpen as you cannot connect
to clients of the same network.

## Downloading Software

The Nvidia SDK Manager is required to download Jetpack.
To download SDK Manager a Nvidia developer account is required.

https://developer.nvidia.com/nvidia-sdk-manager

The SDK Manager (The current latest version is 1.0.0) runs on Ubuntu 
16.04 or 18.04.

Note that for Ubuntu derivative distros like Linux Mint, SDK Manager
fails to detect the OS correctly. This can be fixed by launching with
a environment variable:

```sh
export LSB_ETC_LSB_RELEASE=/etc/upstream-release/lsb-release

sdkmanager
```

## Jetpack

For this setup guide Jetpack 4.3 (Currently the latest) will be used,
but it may be beneficial to use the newest one at the time if this
guide is used in the future.

To get Jetpack, follow the instructions in the SDK Manager. After
the download completes, SDK Manager will prompt you to flash the
Jetson TX1.

**Note:** At the bottom of SDK manager it lists the download location.
In the version used for this guide it was in `~/Downloads/nvidia/sdkm_downloads`.
This directory can be copied between computers to avoid redownloading the files.

## Flashing

On the prompt to flash, make sure to select Manual mode. Then plug the
Micro USB in from your host computer to the the Micro USB port on the
Jetson carrier board. Then follow the instructions to enter recovery mode
and flash.

## OS Setup

**Note: At this point a prompt to install SDKs appears, don't touch it for now**

After the flash completes the Jetson TX1 will still be
in recovery mode and only the serial port will be accessable.

You will need to open the serial port to complete the setup.
The serial port will be at `/dev/ttyACMN` where `N` is a number from 0-9.
You can open the serial port with a utility like putty or picocom (baud rate is 115200):

```sh
$ picocom /dev/ttyACM0 --baud 115200
```

Setup the TX1 with the username/password of choice. It will prompt to setup the network,
however the utility in recovery mode is broken, so just leave the network unconfigured.
After the setup is complete the serial port should show a login, login with the
username/passwork you set and reboot the TX1:

```sh
$ reboot
```

Connect the TX1's HDMI to a monitor and ensure that the Ubuntu login screen appears.

## Installing Nvidia SDKs

At this point the SDKs can be installed. The IP address of the Jetson TX1 is needed to do this.

Find it by opening a terminal on the Jetson and running:

```sh
$ ip addr
```

Input the IP address and username/password set in the previous step into the SDK install.

Then once the SDK is done installing the Jetson is ready to be used!

## Additional Steps

Here are some optional steps for optimizing the TX1.

### Software Update

On the first install the packages in the Jetson TX1 will be out of date. Upgrade them
by running these commands:

```sh
$ sudo apt update && sudo apt upgrade
```

### Expanding Storage using a SATA SSD

The Jetson TX1's internal flash is very small and mostly gets used up by the OS and SDKs.
Fortunately a SATA port is available (SD Card works as well but not very performant).

This guide will use the SSD for `/tmp`, `/home`, and `/opt/ros`.

#### Partitioning the SSD

Connect the SSD to the SATA port on the carrier board. A restart may be needed to fully
detect the SSD.

Open up the Disk Utility (`gnome-disks`) and format the SSD using a GPT partition table.
Add one large partition on the SSD of the `ext4` filesystem (Do NOT use anything else...).

Now to mount note down the UUID of the partition. Now add the following to the end of
`/etc/fstab` using the text editor of choice.

```fstab
UUID=867ec1a4-cd78-4f1c-a172-8e3c6d02feb4    /mnt/ssd    ext4    defaults,noatime,errors=remount-ro    0   1
```

Replace the UUID with the UUID of the partition and make sure to use tabs between the columns rather than spaces.

After the fstab is saved, mount the SSD and begin setup:

```sh
$ sudo mkdir /mnt/ssd
$ sudo mount /mnt/ssd
```

#### Copy files

First, fix the ownership:

```sh
$ sudo chown $USER:$USER -R /mnt/ssd
```

Create the directories:

```sh
$ mkdir /mnt/ssd/home /mnt/ssd/ros /mnt/ssd/tmp
```

Copy every from `/home` to home

```sh
$ cp -a /home/. /mnt/ssd/home/
```

Note: `/opt/ros` should empty since this is a fresh install

Now create the bind mounts by adding these entries to `/etc/fstab`:

```fstab
/mnt/ssd/ros    /opt/ros    none    bind
/mnt/ssd/home    /home    none    bind
/mnt/ssd/tmp    /tmp    none    bind
```

Reboot the apply the changes:

```
$ sudo reboot
```

Done!

#### Installing ROS

If the SSD mounts were done then the Jetson TX1i should have enough space to even
install `ros-melodic-desktop-full` on it.

The install process is the same as on regular Ubuntu on a desktop so just use the
official ROS instructions.
