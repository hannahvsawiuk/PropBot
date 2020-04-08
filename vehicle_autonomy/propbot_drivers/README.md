# PropBot GPS Driver

## Command

Running the following command to operate this package

$ git clone https://github.com/KumarRobotics/ublox.git

$ cd ..

$ catkin_make

$ cd ~/catkin_make/src/ublox/ublox_gps/launch

$ sudo chmod 666 /dev/ttyACM0

$ source ../../../devel/setup.bash

$ roslaunch ublox_device.launch node_name:=gps_param_file nam:=c94_m8p_base

Open another terminal

$ rostopic echo /gps/fix

## More Information

For more information, please refer to https://github.com/KumarRobotics/ublox.git