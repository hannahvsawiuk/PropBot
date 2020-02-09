# PropBot

In order to set up the mission command centre to communicate with the vehicle autonomy computer, perform the following steps:
1. On the vehicle autonomy computer, run: `export ROS_MASTER_URI=http://propbot@vehicle_autonomy_computer:11311`.
2. On the vehicle autonomy computer, run: `roscore`.
3. On the mission command centre computer, run: `export ROS_MASTER_URI=http://propbot@vehicle_autonomy_computer:11311`.

Make sure the export command is either in run in every terminal used.