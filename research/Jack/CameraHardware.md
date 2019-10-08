# Hardware

## Intel Realsense
https://www.intelrealsense.com/compare-depth-cameras/
https://en.wikipedia.org/wiki/Intel_RealSense#Intel_RealSense_D400_Product_Family

We have the top end Realsense in terms of imaging, the D435:

**Depth Sensor**
| Parameter         | Value           |
|        ---        |       ---       |
| Resolution        | 1280 x 720      |
| Video Resolution  | 720p            |
| Video Framerate   | 90 fps          |
| Shutter           | Global          |
| Pixel Size        | 3 micron        |
| FOV               | 85.2 deg Horz   |
|                   | 58 deg Vert     |

**RGB Sensor**
| Parameter         | Value           |
|        ---        |       ---       |
| Resolution        | 1920 x 1080     |
| Video Resolution  | 1080p           |
| Video Framerate   | 30 fps          |
| Shutter           | Global          |
| Pixel Size        | 3 micron        |
| FOV               | 69.4 deg Horz   |
|                   | 42.5 deg Vert   |

There's only the D435i which is "better" but only because it IMU built-in to the camera.

## Regular Cameras

### CCTV Camera

The camera we have uses Analog Video (NTSC/PAL). The Jetson TX-1/Nano doesn't have any hardware
to decode that video, so the camera is useless unless we want to buy a separate decoder.

That's probably not worthwhile since the image quality will be quite low anyways.

### Digital Cameras

https://devtalk.nvidia.com/default/topic/919172/csi-2-cameras-tested-with-tegra-tx1
http://on-demand.gputechconf.com/gtc/2016/webinar/getting-started-jetpack-camera-api.pdf

Looking through some Jetson related tutorials and discussions and it looks like the way to go is to buy a camera with a Video4Linux driver.
It is important that we are able to receive uncompressed video so the frames can be directly processed by the GPU.

Note that the TX-1 also has a hardware video Encoder/Decoder with support for most codecs (h264, h265, vp8, mpeg, ...).
See https://elinux.org/Jetson/H264_Codec (Doesn't mention h265 but it can do it)

#### Interface Options:
How the camera is connected to the Jetson is quite important since the Jetson has fairly limited IO. Here are the options available:
 - MIPI CSI-2 - A common interface used by smartphone cameras, available on both Jetsons as well.
   - The TX-1 can connect up 6 cameras total using this interface, but it requires an expansion board.
   - The Nano only has 2 ports but doesn't need a expansion board to use them.
 - Gigabit Ethernet - Use the ethernet port to receive uncompressed video.
   - We will need a ethernet switch onboard should we choose this option otherwise the camera needs to be unplugged everytime
     we need to log on to the Jetson.
 - USB - Video over USB. This has the most camera options.
   - The TX-1 has only 1 USB 3.0 port and 1 USB 2.0.
   - The Nano for some reason has 4 USB 3.0 ports.

**Additional note about USB:**

The TX-1's single USB 3.0 port is troublesome since the Intel Realsense requires that port.
From personal experience some custom carrier boards have 2 USB 3 ports (E.g. the Auvidea J120) but the second port
can have driver problems causing it to not work for certain devices like the Realsense and possibly any other camera.

To add to the trouble, in order to enable the second port it requires patching the kernel and device tree.
So overall, its best that we assume that the TX-1 has 1 functional USB 3 port even if we plan on buying a
different carrier board. This severely limits camera options since even 1080p video is non-existent for USB 2 cameras.

While the Nano's 4 USB 3 ports is nice, it has half the GPU power compared to the TX-1 which makes it less preferred.

#### General Requirements/Considerations:
 - Global vs Rolling Shutter
   - Rolling shutter is usually cheaper but fast moving objects can be distorted by it.
   - Global shutter is preferred as the robot is on the move, and will have to detect moving objects.
 - Resolution
   - The module should stream 1080p video so atleast 2MP should be sufficient, any higher will put unnecessary load on the CPU/GPU.
     Lower resolution sensors in the same price range have bigger pixels = generally better image quality, low light performance.
 - FOV and Lens
   - Would be nice to have a wider FOV but need to watch out for distortion from a FOV thats too wide (E.g. Fisheye lenses).
     That can really mess up many pre-trained object detection models which expect non-distorted frames.
 - Casing and Form Factor
   - Whether or not the camera comes with a waterproof casing, how easy it is to build a casing for it.

#### Cameras

Nvidia's official partner/recommendation list: https://developer.nvidia.com/embedded/faq#jetson-cameras
elinux compatibility list (Modules verified by Jetson users): https://elinux.org/Jetson/Cameras

A couple of the more interesting modules were chosen for this comparison. The only restriction is that the module is from
an official Nvidia partner claiming TX-1 compatibility or appears on elinux as a module tested to work with the TX-1.

###### Development Kit Camera

The Jetson TX-1 developer kit (which we have) comes with a 5MP Omnivision OV5693 (MIPI CSI) in the box.
The main problem with this camera module is that its uncertain if we can detach it or use a extension cable. If not then
the camera is unusable since it will be stuck to the board making it impossible to waterproof.

**Cost**
Free

**Sensor Specifications**

| Parameter         | Value           |
|        ---        |       ---       |
| Interface         | CSI MIPI x2     |
| Resolution        | 5MP             |
|                   | 2592 x 1944     |
| Video Resolution  | 1080p           |
| Video Framerate   | 30 fps          |
| Shutter           | Rolling         |
| Pixel Size        | 1.4 micron (um) |
| Has Lens          | Yes             |

**Lens Specifications**

Can't find :( https://devtalk.nvidia.com/default/topic/1031939/can-anybody-give-me-a-copy-of-ov5693s-datasheet-/

##### Leopard Imaging, Sony IMX185
https://leopardimaging.com/product/li-jetson-kit-imx185-x/

Leopard Imaging provides a expansion board with 1, 2, 3, or 6 camera modules as a package.
The modules can come with one of two lens mount types: CS or S (M12).

Modules are bare PCBs attached to the expansion board via a decent length cable.

**Cost**
The price ranges from $379 USD (Buy 1) down to $270 USD (Buy 6) per module.

**Sensor Specifications**

| Parameter         | Value           |
|        ---        |       ---       |
| Interface         | CSI MIPI x2/x4  |
| Resolution        | 2MP             |
|                   | 1952 x 1241     |
| Video Resolution  | 1080p           |
| Video Framerate   | 30 fps (MIPI x2)|
|                   | 60 fps (MIPI x4)|
| Shutter           | Rolling         |
| Pixel Size        | 3.75 micron     |
| Has Lens          | Yes             |

**Lens (CS Mount) Specifications**

| Parameter         | Value           |
|        ---        |       ---       |
| Mount Type        | CS              |
| FOV               | 102 deg Diag    |
|                   | 90 deg Horz     |
|                   | 50 deg Vert     |
| Aperture          | f/2.2           |
| Focal Length      | 5 mm            |
| TV Distortion     | -8 %            |

**Lens (S Mount) Specifications**

| Parameter         | Value           |
|        ---        |       ---       |
| Mount Type        | S               |
| FOV               | 102 deg Diag    |
|                   | 92 deg Horz     |
|                   | 60 deg Vert     |
| Aperture          | f/2.8           |
| Focal Length      | 5 mm            |
| TV Distortion     | -1 %            |

##### e-con Systems e-CAM31_TX1, On Semi AR0330
https://www.e-consystems.com/jetson-tx2-ultra-low-light-camera-board.asp

Comes with 1 sensor plus adapter board. Claims great low light performance.

The sensor (Bare PCB) might be stuck to the adapter board. (Can't see in images)

Annoyingly enough the datasheet is locked behind a registration.

**Cost**
The one module and board cost $179 USD

**Sensor Specifications**

| Parameter         | Value           |
|        ---        |       ---       |
| Interface         | CSI MIPI x4     |
| Resolution        | 2MP             |
|                   | 1945 x 1097     |
| Video Resolution  | 720p, 1080p     |
| Video Framerate   | 60 fps          |
| Shutter           | Rolling         |
| Pixel Size        | 2.9 micron      |
| Has Lens          | Yes             |

**Lens Specifications**
Cannot access datasheet

##### D3 Engineering, Rugged Sony IMX390-953
https://store.d3engineering.com/product/designcore-d3rcm-imx390-953-rugged-camera-module/

The main feature of this module is the rugged casing which is waterproof.

Uses a interface called FPD-Link III which appears to be related to (Or just is) MIPI?
Overall its unclear how to connect it to the TX-1 but D3 Engineering specifically advertises
it as compatible.

The camera has two lens options with different Horizontal FOVs: 70 and 192 degress.
The 192 degree lens is probably a non-starter since the FOV is insane which will certainly cause
huge distortion.

**Cost**
$449 USD

**Sensor Specifications**

| Parameter         | Value           |
|        ---        |       ---       |
| Interface         | CSI MIPI ?????  |
| Resolution        | 2MP             |
|                   | 1920 x 1200     |
| Video Resolution  | 1080p           |
| Video Framerate   | 60 fps          |
| Shutter           | Rolling         |
| Pixel Size        | 3.0 micron      |
| Has Lens          | Yes             |

**Lens Specifications**

Sunex DSL958

| Parameter         | Value           |
|        ---        |       ---       |
| Mount Type        | S               |
| FOV               | 76 deg Diag     |
|                   | 70 deg Horz     |
| Aperture          | f/2.8           |
| Focal Length      | 4.6 mm          |
| TV Distortion     | 17% rectilinear |

##### Flir/Point Grey Blackfly S GigE w/ Sony IMX430 Color
https://www.flir.com/products/blackfly-s-gige/
Really good quality sensor, but costs a lot.

Uses C-Mount Lenses which need to be purchased separately.

The casing looks pretty rugged, unsure if waterproof though.

A couple of honorable mentions:
Blackfly - Very similar spec sensors (Non-Sony) available at similar cost.
Chameleon3 - Some very cheap options available, unfortunately USB 3 only.
USB2 Cameras - A 1.3MP may be a good option if it is cheap, unfortunately
               prices are not mentioned on Flir's site.

**Cost**
$545 USD and it doesn't come with a lens...

**Sensor Specifications**

| Parameter         | Value           |
|        ---        |       ---       |
| Interface         | Gig Ethernet POE|
| Resolution        | 2MP             |
|                   | 1616 x 1240     |
| Video Resolution  | 1080p           |
| Video Framerate   | 60 fps          |
| Shutter           | Global          |
| Pixel Size        | 4.5 micron      |
| Has Lens          | No              |

##### Allied Vision Manta G-201
https://www.alliedvision.com/en/products/cameras/detail/Manta/G-201.html

Has some pricing information since Allied Vision doesn't give any.
https://www.edmundoptics.com/f/allied-vision-manta-gige-cameras/14149/

**Very** costly, easily double even the Flir cameras at a similar resolution.

Uses CCD sensors instead of CMOS, and comes in a nice somewhat large metal casing.

Can use C, CS or M12 Lenses sold separately....

**Cost**
This model costs $1650 USD from Edmund Optics. Oof.

**Sensor Specifications**
Sony ICX274

| Parameter         | Value           |
|        ---        |       ---       |
| Interface         | Gig Ethernet POE|
| Resolution        | 2MP             |
|                   | 1624 x 1240     |
| Video Resolution  | 1080p           |
| Video Framerate   | 14.7 fps        |
| Shutter           | Global          |
| Pixel Size        | 4.4 micron      |
| Has Lens          | No              |

#### Conclusions

The recommendation of which camera really depends on the use case.
If we plan on mounting multiple cameras the Leopard Imaging boards are a good value option that will just work.
If it turns out image quality is super crucial, the Flir is probably the best option.
