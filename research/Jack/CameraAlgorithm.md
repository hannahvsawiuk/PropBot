# Camera

## General

It seems like right now we have both the Realsense camera as well as a Lidar and the possibility to use a regular camera.

- Realsense
 - Provides point cloud infront of the camera
 - Provides color video to fuse with the point cloud

- Lidar
 - Provides a point cloud 360 degrees around (Not very large vertical arc for our Lidar)

- Camera (Security Camera or other Camera)
 - Provides color video.

These seem to be overlapping too much if we try to use both for forward vision. If we combine the Lidar information with the object detection of a regular camera it does
the same task as the Realsense. So we aren't necessarily getting more information but we are doing double the computation. There might be a slight accuracy advantage of using
both but the disadvantage of the extra computing might make that accuracy advantage small by comparison.

##### Possible Solutions:

Use only one:
If we had to only use 1 solution the best would be to stick with the Lidar + Camera as that will still provide 360 object detection incase of bikes/cars coming up behind the robot.

###### Use both but for different purposes:

The best use of the hardware would be to use the Lidar + Camera for long range detection of fairly tall objects (People walking, cars, buidings, etc) then use
the depth camera to scan the ground pointing at a lower angle.

This makes good use of both since:
- The Lidar cannot see the ground close to the robot (or at all) where it will be mounted. Maybe a higher end model would be able to.
- Realsense can provide high resolution point clouds which is suitable for mapping out the terrain

## Algorithms

#### General Object Detection Algorithms:
https://towardsdatascience.com/beginners-guide-to-object-detection-algorithms-6620fb31c375
https://towardsdatascience.com/r-cnn-fast-r-cnn-faster-r-cnn-yolo-object-detection-algorithms-36d53571365e

- R-CNN (Regular, Fast, Faster)
  - Super slow (CUDA might help if theres a compatible implementation)
  - Accurate
- YOLO
  - Fast
  - Less accurate for small objects, probably not a concern though
- SSD
  - between YOLO and R-CNN in speed and accuracy
- R-FCN
  - Faster than R-CNN Fast (Still quite slow?)

OpenCV has a implementation of all of these algorithms.

YOLO seems like the best for us since we want fast detection rate. Most of our
objects will be in the open (Car on road, person walking on the side walk) and fairly large.
Anything else like a person/car very far away isn't worth detecting since it is not a immediate
danger until its closer to the robot.

Darknet is a GPU Accelerated implementation of YOLO https://pjreddie.com/darknet/.
This should be suitable for the Jetson TX-1/Nano as they have NVidia GPUs onboard.

#### Depth Cameras:
Depth images can provide similar data to a LiDAR. A depth image (Z) combined with the camera's FOV info
can be converted into a point cloud (A collection of X, Y, Z points). Point Cloud Library is a open source
library which can perform these types of operations.

http://www.pointclouds.org/about/

This paper talks about different algorithms for use on embedded systems and compares their performance.
Can't say much about them as I understood quite little of it.
https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?referer=&httpsredir=1&article=3971&context=etd

