## Hardware

### Intel Realsense
https://www.intelrealsense.com/compare-depth-cameras/
https://en.wikipedia.org/wiki/Intel_RealSense#Intel_RealSense_D400_Product_Family

We have the top end Realsense (D435) in terms of imaging.

HFOV: 85.2 degrees
VFOV: 58 degrees

There's a one tier higher (D435i)
one which the only difference is that it has a IMU builtin to the camera.

### Regular Cameras

#### CCTV Camera

The camera we have uses Analog Video (NTSC/PAL). The Jetson TX-1/Nano doesn't have any hardware
to decode that video, so the camera is useless unless we want to buy a separate decoder.

That's probably not worthwhile since the image quality will be quite low anyways.

#### Digital Cameras

https://elinux.org/Jetson_Nano#Cameras

The Jetson Nano has a camera interface (Works with Raspberry Pi Compatible cameras).
The Jetson TX-1 requires a separate daughter board to get this interface.

For reference a 8MP 1080P Raspberry Pi Camera Module is $33 off Amazon.

https://www.amazon.ca/Raspberry-Pi-Camera-Module-Megapixel/dp/B01ER2SKFS

A new 12MP 1080P USB Webcam can be had for about that price as well.

https://www.amazon.ca/AUSDOM-1920x1080P-Noise-cancelling-Recording-Computer/dp/B017SRHRR6/ref=sr_1_9?crid=29SKXSWQY4TMV&keywords=usb+webcam+1080p&qid=1569927136&s=electronics&sprefix=usb+we%2Celectronics%2C222&sr=1-9

As a final option Avigilon sells security cameras (H5A lineup) with super
accurate real time object detection. The main downside being that a single
camera costs around $1K, and they may/may not sell to us.

