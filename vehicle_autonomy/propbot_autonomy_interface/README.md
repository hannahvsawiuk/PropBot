# PropBot

## Installing Arduino Rosserial Library

Using the following commands to install Arduino Rosserial Library:

$ sudo apt-get install ros-melodic-rosserial-arduino ros-melodic- rosserial-embeddedlinux ros-melodic-rosserial-windows ros-melodic- rosserial-server ros-melodic-rosserial-python

$ cd ~/catkin_ws/src/

$ git clone https://github.com/ros-drivers/rosserial.git

$ cd ~/catkin_ws/

$ catkin_make

Download Arduino IDE

$ cd ~/Arduino/libraries/

$ rosrun rosserial_arduino make_libraries.py

## Running This ROS Package

Put the arduino.launch file into the rosserial_python directory.

$ sudo chmod 666 /dev/ttyUSB0 (**Or other port's name, such as ttyACM0,ttyACM1. Check your port's name before this command, the default name is ttyUSB0. If your port's name is a different one, you will also need to modify the port parameter in the arduino.launch**)

$ roslaunch rosserial_python arduino.launch

## Sending Wheel Commands in ROS

Keep the roslaunch command above running and open up a new termial. The following command gives you an example to send wheel commands in ROS:

$ rostopic pub -r 5 listener_right std_msgs/UInt8 "5"

## Revealing the Wheel Commands on Screen

Keep the roslaunch command above running and open up a new terminal. The following command gives you an example to reveal wheel commands on screen:

$ rostopic echo /rw

## More Information

For more information, please download this book and refer to **Page 165-171**

http://www.mediafire.com/file/o59o7gnruwoxdod/ROBOT_OPERATING_SYSTEM_COOKBOOK.zip/file

