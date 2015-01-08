(mklingen)
A note on controllers: apparently, Ada requires `velocity_controller/JointTrajectoryController` which does not exist in hydro. That means you must check out and build `ros_controllers` from [ROS indigo](https://github.com/ros-controls/ros_controllers) from source.
