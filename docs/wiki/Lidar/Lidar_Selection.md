## Considerations for Choosing a LiDAR for Outdoor Robots

* resistance to ambient light
    * intense sunlight can prevent scanner from being able to read its own returning light pulses
    * choose LiDAR with high sunlight resistance 
* resistance to Environmental Noise
    * environmental factors such as precipitation can interfere with LiDARs ability to detect obstacles
    * must maintain high levels of accuracy in conditions such as rain, snow etc.
    * possible with multi-echo technology (for i.e. SICK laser scanners) 
        * part of the energy from a pulse may be reflected by objects like rain, while the remainder of the beam continues to propagate and is reflected by actually obstacle
        * multiple "echos" can be recognized by LiDAR, and the weaker reflections can be ignored
* high environmental rating
    * for i.e. IP 67 rating will ensure the reliability of laser scanner even if it is immersed in water
    * able to cope with dust and grime: sends multiple pulses at slight increments and filter the returning data to subtract dust 
* temperature range
    * look for scanners with wide operating temperature range and built-in temperature control system
* electromagnetic (EMI) considerations
    * electromagnetic noise can cause robot to behave erratically
    * less of a consideration for Propbot
* Update rate
    * a robot is travelling 2m/s (4.5 mph) would need a laser that can see at least double that range at a refresh rate of 10Hz or greater 
* specs can sometimes be over-optimistic
    * range specifications can sometimes be measured using retro-reflective targets
    * rule of thumb is the halve the quoted range



## Comparison Table
|  Manufacturer | Product | Distance (m) | rotation rate (Hz) | single return sample rate (points/sec) | dual return mode (points/sec) | evaluated echos | Horizontal ang. Res (deg) | Vert. ang. Res. (deg) | Horizontal FOV<br/>(deg) | Vertical FOV (deg) | environmental protection | Power (W) | Point density on human figure (1.7 x 0.6m) 5 m away | price |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
|  Slamtec | [RPLiDAR](https://www.slamtec.com/en/Lidar/A3 "RPLiDAR") | 20 | 10 to 20 | 16000 or 10000 |  |  | 0.225 or 0.36 | N/A | 360 | N/A |  | 3 |  | 800 |
|  Velodyne | [HDL-32E](https://www.velodynelidar.com/hdl-32e.html "HDL-32E") | 80 to 100 | 5 to 20 | 695000 | 1390000 | 2 | 0.08 to 0.33 | 1.33 | 360 | 10.67 to -30.67 | IP67 | 12 |  | 10000+ |
|  Velodyne | [puck](https://www.velodynelidar.com/vlp-16.html "puck") | 100 | 5 to 20 | 300000 | 600000 | 2 | 0.1 to 0.4 | 2 | 360 | 15 to -15 | IP67 | 8 |  | 8000 |
|  SICK | [MRS1104C-111011](https://www.sick.com/ca/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs1000/mrs1104c-111011/p/p495044?ff_data=JmZmX2lkPXA0OTUwNDQmZmZfbWFzdGVySWQ9cDQ5NTA0NCZmZl90aXRsZT1NUlMxMTA0Qy0xMTEwMTEmZmZfcXVlcnk9JmZmX3Bvcz0xJmZmX29yaWdQb3M9MSZmZl9wYWdlPTEmZmZfcGFnZVNpemU9OCZmZl9vcmlnUGFnZVNpemU9OCZmZl9zaW1pPTkyLjA= "MRS1104C-111011") | 30 | fixed | 55000 to 165000 |  | 3 | 0.25 |  | 275 | 7.5 over 4 layers | IP67 | <13 |  |  |
|  SICK | [MRS6124R-131001](https://www.sick.com/ca/en/detection-and-ranging-solutions/3d-lidar-sensors/mrs6000/mrs6124r-131001/p/p533545?ff_data=JmZmX2lkPXA1MzM1NDUmZmZfbWFzdGVySWQ9cDUzMzU0NSZmZl90aXRsZT1NUlM2MTI0Ui0xMzEwMDEmZmZfcXVlcnk9JmZmX3Bvcz0xJmZmX29yaWdQb3M9MSZmZl9wYWdlPTEmZmZfcGFnZVNpemU9MjQmZmZfb3JpZ1BhZ2VTaXplPTI0JmZmX3NpbWk9OTMuMA== "MRS6124R-131001") | 75 | fixed | 880000 |  | 4 | 0.13 | 0.625 | 120 | 15 | IP67 | 20 |  |  |
