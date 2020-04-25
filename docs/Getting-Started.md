# Getting Started

---

## Table of Contents

- [ROS Installation and Workspace Setup](#ros-installation-and-workspace-setup)
  * [Installing ROS Melodic](#installing-ros-melodic)
      - [A note on catkin](#a-note-on-catkin)
  * [Create the workspaces](#create-the-workspaces)
    + [Cartographer workspace](#cartographer-workspace)
    + [Propbot workspace](#propbot-workspace)
  * [Build an Optimized Google Cartographer for ROS](#build-an-optimized-google-cartographer-for-ros)
    + [Run the custom Cartographer setup script](#run-the-custom-cartographer-setup-script)
    + [Build the Cartographer workspace](#build-the-cartographer-workspace)
  * [Setup the Propbot Workspace](#setup-the-propbot-workspace)
    + [Make Propbot an extension of Cartographer](#make-propbot-an-extension-of-cartographer)
    + [Install dependencies](#install-dependencies)
      - [A note on rosdep](#a-note-on-rosdep)
    + [Build the mapviz plugins](#build-the-mapviz-plugins)
    + [Build the workspace](#build-the-workspace)
- [Running the Demos](#running-the-demos)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

---

## ROS Installation and Workspace Setup

The following goes over how to install ROS and setup the catkin workspaces

### Installing ROS Melodic

Follow the instructions here to install the `melodic` version of ROS: http://wiki.ros.org/melodic/Installation/Ubuntu

Ensure that you install the **Desktop-Full Install** version.

##### A note on catkin

Documentation: http://wiki.ros.org/catkin

`catkin` is the build system used for ROS. ROS comes with `catkin`. After doing the installation of ROS, run the following command:

```bash
catkin --version
```

You should see an output similar to this:

![catkin_version](graphics/catkin_version_response.png)

If you do not, there may have been an issue installing catkin. To resolve this, run the following command and try the version check command again:

```bash
sudo apt-get install ros-kinetic-catkin python-catkin-tools
```

### Create the workspaces

#### Cartographer workspace

In your home directory, make the cartographer workspace directory

```bash
cd ~/
mkdir cartographer_ws
```

#### Propbot workspace

In your home directory, make the `propbot` workspace directory

```bash
cd ~/
mkdir propbot_ws
```

Then, make the `src` directory and clone the Propbot repo into the new `src` directory.

```bash
cd ~/propbot_ws
mkdir src
cd src
git clone https://github.com/hannahvsawiuk/PropBot.git
```

### Build an Optimized Google Cartographer for ROS

Now that ROS has been installed, we need to start by building Google Cartographer for our application. Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.

**Cartographer documentation:**

- General information: https://google-cartographer.readthedocs.io/en/latest/
- Integration with ROS: https://google-cartographer-ros.readthedocs.io/en/latest/

#### Run the custom Cartographer setup script

The script to setup the custom version of Cartographer is in `vehicle_autonomy/scripts`. To run the setup for the `cartographer_ws`, run the following command:

```bash
~/propbot_ws/src/PropBot/vehicle_autonomy/scripts/setup_cartographer_build.sh ~/cartographer_ws
```

If everything runs smoothly, you should see the following when the process finishes

![cartographer_setup](graphics/cartographer_setup_success.png)

#### Build the Cartographer workspace

Run the following to initialize and build the workspace

```bash
cd ~/cartographer_ws
catkin build
```

This will take some time (up to **30 minutes**), so please be patient. If the build succeeds, you should see the following statements:

![cartographer_build](graphics/cartographer_build_success.png)

### Setup the Propbot Workspace

Now that the custom Cartographer version has been built, it will be extended with the Propbot packages.

#### Make Propbot an extension of Cartographer

To initialize the workspace and indicate that `propbot_ws` will be an extension of `cartographer_ws`, run the following commands

```bash
cd ~/propbot_ws
catkin init
catkin config --extend ../cartographer_ws/devel
```

The `--extend` argument explicitly sets the workspace you want to extend. In this case, we want the packages in propbot to extend the custom cartographer setup.

For more information on `catkin config`, see here: https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html

#### Install dependencies

To ensure that everything runs smoothly, the dependencies specified by the Propbot packages need to be installed. To accomplish this, run the following:

**Ubuntu**

```bash
cd ~/propbot_ws
rosdep install --from-paths src --ignore-src
```

**Alternative Distros**
If you are not using Ubuntu, the `--os` argument is needed, e.g. `--os=ubuntu:bionic`:

```bash
cd ~/propbot_ws
rosdep install --os=<distro info> --from-paths src --ignore-src
```

##### A note on rosdep

`rosdep` is a dependency manager for ROS that helps you install external dependencies in an OS-independent manner.

**`rosdep` documentation:**

- Wiki: http://wiki.ros.org/rosdep
- Command reference: https://docs.ros.org/independent/api/rosdep/html/commands.html

#### Build the mapviz plugins

Prior to building the entire `propbot_ws`, the mapviz plugs need to be built. Run the following

```bash
catkin build mapviz_plugins
```

If successful, you should see the following

![mapviz_build](graphics/mapviz_build_success.png)

#### Build the workspace

Finally, build the entire `propbot_ws`:

```bash
cd ~/propbot_ws
catkin build
```

Ensure that all packages have been successfully built. You should see the following if the build was completed

![propbot_ws_build](graphics/propbot_ws_build_success.png)

Now your `propbot_ws` directory should have the following structure

```
├── build/          # Build space
├── devel/          # Devel space
├── logs/           # Logs space
└── src/            # Source space
    └── Propbot/    # Propbot packages

```

For more information on catkin workspace mechanics, see here: https://catkin-tools.readthedocs.io/en/latest/mechanics.html

---

## Running the Demos

These instructions describe how to set-up and run an autonomous waypoint mission using Propbot. If you are not executing the mission in simulation, skip the instructions for _Setting up the simulation computer_. 


### Setting up communication between multiple machines

In order to set up the mission command centre to communicate with the vehicle autonomy computer, perform the following steps:

1. Make sure both the mission command centre computer and vehicle autonomy computer are on the same network. Do a simple ping test to 
make sure they can communicate with each other.
2. On the vehicle autonomy computer, run: `export ROS_MASTER_URI=http://propbot@vehicle_autonomy_computer:11311`.
3. On the vehicle autonomy computer, run: `roscore`.
4. On the mission command centre computer, run: `export ROS_MASTER_URI=http://propbot@vehicle_autonomy_computer:11311`.

Make sure the export command is either in run in every terminal used.

Repeat steps all of the above steps to set up communication between the simulation computer and vehicle autonomy computer. 

### Setting up the simulation computer

On the simulation computer, build all of the packages inside the _simulation_ directory.

```bash
catkin build  propbot_gazebo  propbot_rviz  propbot_simulation
```

The following commands will a launch simulation environment that is modeled off of this area on UBC campus.

![propbot_sim_area](graphics/google_ubc_student_union.png) 

If you would like use a static simulation environment launch the following file:
```bash
roslaunch propbot_simulation ubc_student_union_sim.launch
```

If you would like to use a dynamic simulation environment launch this file instead of the one above:
```bash
roslaunch propbot_simulation ubc_student_union_sim_people.launch
```
### Setting up the vehicle autonomy computer


Build all of the packages inside the _vehicle_autonomy_ and _common_ directory:


```bash
catkin build propbot_autonomy  propbot_control  propbot_description  propbot_drivers  propbot_mission  propbot_navigation  propbot_slam  propbot_state_estimation propbot_common_msgs  propbot_util
```
Launch the autonomy stack:
```bash
roslaunch propbot_autonomy autonomy.launch
```


### Setting up the mission command centre

Build all of the packages inside the _mission_command_centre_ and _common_ directory:

```bash
catkin build  mapviz_plugins  propbot_mission_gui propbot_common_msgs  propbot_util
```

Launch the mission command centre GUI:
```bash
roslaunch propbot_mission_gui mapviz.launch
```

![mapviz_gui](graphics/mapviz_GUI.png) 

The above shows the default GUI configuration. If you would like to display satellite map tiles follow the instructions from this repository

**ROS Offline Google Maps for MapViz:** https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite


#### Executing the mission

Configure and command a mission using the _plan_mission_ section of the control panel:
1. Select mission waypoints by clicking on the map and clear them as you like by selecting _Clear_
2. Once you are satisfied with your mission configuration, upload the mission to propbot by selecting _Upload Mission_
3. Start the mission by selecting _Start Mission_
4. You may pause, resume, or end the mission at anytime by selecting _Pause Mission_, _Resume Mission_, and _End Mission_
5. Monitor the robots progress on the display 

**Note**: You must wait until a mission is finished or end the mission before uploading a new one



![mapviz_waypoint](graphics/mapviz_waypoint.png) 


