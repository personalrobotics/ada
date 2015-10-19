# Ada
Ada (or Assistive Dextrous Arm) is a package for controlling the Mico robot arm through the Personal Robotics pipeline. It is a high-level package, pulling from ROS, OpenRAVE, and other lower-level parts of the system. At the highest level, it provides scripts for controlling the robot, planning trajectories, finding inverse kinematics solutions, and more.

## Installation ##
The following rosinstall can be used to get the minimum dependencies required
to use HerbPy in simulation:
```shell
$ wstool merge https://raw.githubusercontent.com/personalrobotics/pr-rosinstalls/master/ada-sim.rosinstall
```
If you plan to connect to the real robot, then you will need a larger set of
dependencies:
```shell
$ wstool merge https://raw.githubusercontent.com/personalrobotics/pr-rosinstalls/master/ada.rosinstall
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
