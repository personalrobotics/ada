# Ada
Ada (or Autonomous Dextrous ... a?) is a package for controlling the Mico robot arm through the Personal Robotics pipeline. It is a high-level package, pulling from ROS, OpenRAVE, and other lower-level parts of the system. At the highest level, it provides scripts for controlling the robot, planning trajectories, finding inverse kinematics solutions, and more.

# Installing The full software pipeline in ROS indigo and 140.04
A. **Install Dependencies**
    Before you start, you will need to install the following debians:
    
    ```
    libnewmat10-dev
    liblapacke-dev
    ipython
    libnlopt-dev
    libsoqt-dev
    libode-dev
    libgeos-dev
    gsl-bin
    ```
    
    You will also need the following python dependencies:
    
    ```
    numpy
    scipy
    ```

1. **Install ROS**

    To install ROS, follow [these stesp](http://wiki.ros.org/indigo/Installation/Ubuntu). Ada will work on ROS hydro     or indigo. These instructions are for indigo. You will want to install the following ROS debians:
  ```
  ros-indigo-desktop-full 
  ros-indigo-ros-control 
  ros-indigo-ros-controllers 
  ros-indigo-ompl 
  ros-indigo-srdfdom
  ```

  > A note on controllers: Ada requires `velocity_controller/JointTrajectoryController` which does not exist in hydro. That means you must check out and build `ros_controllers` from [ROS indigo](https://github.com/ros-controls/ros_controllers) from source if you are running ros hydro.

2. **Create a `catkin` workspace**

  Create a folder for your catkin workspace. We called it `~/pr_catkin/`. Then, create a folder inside it called `src`.   Then, run the following:

  ```
  source /opt/ros/indigo/setup.bash
  cd ~/pr_catkin/src
  catkin_init_workspace
  cd ..
  catkin_make
  ```
  This will generate a `catkin` source tree for you, which all the code will live in.

3. **Create a `.rosinstall` file**

  Now we will define a `.rosinstall` file that will define all of the github repositories we will need. Create the file `~/pr_catkin/src/.rosinstall` with the following contents:
  ```  
  - git: {local-name: openrave, uri: 'git@github.com:personalrobotics/openrave.git'}
  - git: {local-name: openrave_catkin, uri: 'git@github.com:personalrobotics/openrave_catkin.git'}
  - git: {local-name: comps, uri: 'git@github.com:personalrobotics/comps.git'}
  - git: {local-name: prpy, uri: 'git@github.com:personalrobotics/prpy.git'}
  - git: {local-name: or_cdchomp, uri: 'git@github.com:personalrobotics/or_cdchomp.git'}
  - git: {local-name: or_rviz, uri: 'git@github.com:personalrobotics/or_rviz.git'}
  - git: {local-name: or_ompl, uri: 'git@github.com:personalrobotics/or_ompl.git'}
  - git: {local-name: ada, uri: 'git@github.com:personalrobotics/ada.git'}
  - git: {local-name: mico_hardware, uri: 'git@github.com:personalrobotics/mico_hardware.git'}
  - git: {local-name: openrave, uri: 'git@github.com:personalrobotics/openrave.git'}
  - git: {local-name: or_nlopt_ik, uri: 'git@github.com:personalrobotics/or_nlopt_ik.git'}
  - git: {local-name: or_urdf, uri: 'git@github.com:personalrobotics/or_urdf.git'}
  - git: {local-name: pr-ordata, uri: 'git@github.com:personalrobotics/pr-ordata.git'}
  - git: {local-name: or_ros_controller, uri: 'git@github.com:personalrobotics/or_ros_controller.git'}
  - git: {local-name: libkindrv, uri: 'git@github.com:personalrobotics/libkindrv.git'}
  - git: {local-name: pr_ros_controllers, uri: 'git@github.com:personalrobotics/pr_ros_controllers.git'}
  ```
  
  Now, we tell ROS to pull all of these github repositories into our source tree by doing:
  
  ```
  cd ~/pr_catkin/src
  wstool up
  ```
4. **Install Openrave**

  For 14.04/indigo, you must install OpenRAVE from source. Luckily, it was pulled already by the `.rosinstall` file. To build and install openrave do this:
  
  ```
  cd ~/pr_catkin/src/openrave
  make
  sudo make install
  ```
5. **Build the Code**

    To build the code, just do this:
    ```
    cd ~/pr_catkin
    source ./devel/setup.bash
    catkin_make
    ```
6. **Connect to the robot**

  To connect to the robot, run the launch script:
  
  ```
  roslaunch ada_launch default.launch
  ```
  
  You may have to re-launch several times before the robot will connect. This will also start up all the controllers.
  
7. **Run a test script**

   After connecting to the robot, open up a new terminal and set it up:
   ```
   source /opt/ros/indigo/setup.bash
   source ./devel/setup.bash
   ```
   You will need to do this every time you open up a new terminal. `devel/setup.bash` will have to be run any time the catkin workspace is rebuilt.
   
   Now, run the test script:
   
   `rosrun adapy test.py`

8. **Play around**

  This will open up a python console that you can use to send commands to the robot. See `prpy` for more info.
