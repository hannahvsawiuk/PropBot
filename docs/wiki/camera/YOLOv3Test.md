# Test
So a simple test is done to evaluate the Jetson TX1's performance.
YOLOv3 is chosen as it is a very efficient object detection algorithm, see the paper: https://arxiv.org/abs/1804.02767.

## Setup

Installation was done by compiling natively on the Jetson TX1. Nvidia has CUDA and OpenCV set up already onboard so its fairly easy to get things working.

Darknet is installed as per the instructions at https://pjreddie.com/darknet/install/.
Make sure to enable CUDA and OpenCV as they are both needed for the live video demo.

Then following instructions at https://pjreddie.com/darknet/yolo/, download the YOLOv3 Tiny Weights.

## Running

The detector demo is run using the yolov3 tiny weights. Specify the correct video device (/dev/videoN) using the -c argument (If unsure use VLC to check all devices).

```
$ ./darknet detector demo cfg/coco.data cfg/yolov3-tiny.cfg yolov3-tiny.weights -c N
```

## Results

The YOLOv3 demo runs at about 12-16 FPS on the Intel Realsense camera presumably at full resolution (1080p). Performance gains can be expected by scaling the video
to lower resolutions (E.g. 720p or 480p) at the cost of lower detection accuracy. This was not tested as it requires some non-trivial code modifications.

Note that this test was also done on the YOLOv3 regular weights but as soon as the test started the Jetson ran out of RAM and became unrecoverable without rebooting.
It is likely that the regular weights have better detection accuracy, there are some fairly noticable problems with the detection accuracy using the tiny weights.

Running the demo uses about 25-30% CPU on all 4 cores of the Jetson TX1. At the moment it is unclear if this is from the neural network or the demo program.
However is is certain that this kind of CPU overhead is not acceptable since it limits our ability to perform other tasks on the Jetson (E.g. LiDAR data processing).
