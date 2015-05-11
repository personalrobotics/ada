# Ada
Ada (or Assistive Dextrous Arm) is a package for controlling the Mico robot arm through the Personal Robotics pipeline. It is a high-level package, pulling from ROS, OpenRAVE, and other lower-level parts of the system. At the highest level, it provides scripts for controlling the robot, planning trajectories, finding inverse kinematics solutions, and more.

## Installation (Ubuntu 14.04) ##

Before you start, you will need to install the following system dependencies:

- libnewmat10-dev
- liblapacke-dev
- ipython
- libnlopt-dev
- libsoqt-dev
- libode-dev
- libgeos-dev
- libgsl0-dev

You will also need the following Python dependencies:

- numpy
- scipy

To install all of these using Apt, run:

```bash
apt-get install libnewmat10-dev liblapacke-dev ipython libnlopt-dev libsoqt-dev libode-dev libgeos-dev libgsl0-dev python-numpy python-scipy
```

### Install ROS ###

To install ROS, follow [these steps](http://wiki.ros.org/indigo/Installation/Ubuntu). Ada will work on ROS hydro     or indigo. These instructions are for indigo. You will want to install the following ROS debians:

- ros-indigo-desktop-full
- ros-indigo-ros-control
- ros-indigo-ros-controllers
- ros-indigo-ompl
- ros-indigo-srdfdom

To install these, run:

```bash
apt-get install ros-indigo-ros-control ros-indigo-ros-controllers ros-indigo-ompl ros-indigo-srdfdom
```

  > A note on controllers: Ada requires `velocity_controller/JointTrajectoryController` which does not exist in hydro. That means you must check out and build `ros_controllers` from [ROS indigo](https://github.com/ros-controls/ros_controllers) from source if you are running ros hydro.

### Create a `catkin` workspace ###

Create a folder for your catkin workspace. We called it `~/pr_catkin/`. Then, create a folder inside it called `src`.   Then, run the following:

```bash
source /opt/ros/indigo/setup.bash
cd ~/pr_catkin/src
catkin_init_workspace
cd ..
catkin_make
```

This will generate a `catkin` source tree for you, which all the code will live in.

### Create a `.rosinstall` file ###

Now we will define a `.rosinstall` file that will define all of the github repositories we will need. Create the file `~/pr_catkin/src/.rosinstall` with the following contents:

```yaml
- git: {local-name: openrave, uri: 'git@github.com:personalrobotics/openrave.git'}
- git: {local-name: openrave_catkin, uri: 'git@github.com:personalrobotics/openrave_catkin.git'}
- git: {local-name: comps, uri: 'git@github.com:personalrobotics/comps.git'}
- git: {local-name: prpy, uri: 'git@github.com:personalrobotics/prpy.git'}
- git: {local-name: or_cdchomp, uri: 'git@github.com:personalrobotics/or_cdchomp.git'}
- git: {local-name: or_rviz, uri: 'git@github.com:personalrobotics/or_rviz.git'}
- git: {local-name: or_ompl, uri: 'git@github.com:personalrobotics/or_ompl.git'}
- git: {local-name: ada, uri: 'git@github.com:personalrobotics/ada.git'}
- git: {local-name: mico_hardware, uri: 'git@github.com:personalrobotics/mico_hardware.git'}
- git: {local-name: or_nlopt_ik, uri: 'git@github.com:personalrobotics/or_nlopt_ik.git'}
- git: {local-name: or_urdf, uri: 'git@github.com:personalrobotics/or_urdf.git'}
- git: {local-name: pr-ordata, uri: 'git@github.com:personalrobotics/pr-ordata.git'}
- git: {local-name: or_ros_controller, uri: 'git@github.com:personalrobotics/or_ros_controller.git'}
- git: {local-name: libkindrv, uri: 'git@github.com:personalrobotics/libkindrv.git'}
- git: {local-name: pr_ros_controllers, uri: 'git@github.com:personalrobotics/pr_ros_controllers.git'}
```
  
Now, we tell `wstool` to pull these repositories by running:
  
```bash
cd ~/pr_catkin/src
wstool up
```

### Install OpenRAVE ###

  For 14.04/indigo, you must install OpenRAVE from source. Luckily, it was pulled already by the `.rosinstall` file. To build and install OpenRAVE do this:
  
```bash
cd ~/pr_catkin/src/openrave
make
sudo make install
```

### Build the Code ###

To build the code, just do this:
```bash
cd ~/pr_catkin
source ./devel/setup.bash
catkin_make
```

### Set up the udev rules ###

Create a file called `/etc/udev/rules.d/45-jaco.rules` with the content:

```
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor} =="22cd", MODE="0666", GROUP="pr", SYMLINK+="mico"
```

> A note on USB 3.0: the mico arm does not work with USB 3.0 ports. If your computer has no USB 2.0 ports, you will need to disable xHCI in your BIOS.

### Connect to the robot ###

To connect to the robot, run the launch script:

```bash
roslaunch ada_launch default.launch
```
  
You may have to re-launch several times before the robot will connect. This will also start up all the controllers.
  
### Run a test script ###

After connecting to the robot, open up a new terminal and set it up:
```bash
. ./devel/setup.bash
```

You will need to do this every time you open up a new terminal. `devel/setup.bash` will have to be run any time the catkin workspace is rebuilt.
   
Now, run the test script:
   
```bash
rosrun adapy test.py
```

### Play around ###

This will open up a python console that you can use to send commands to the robot. See `prpy` for more info.
