# Open_manipulator_6dof_application

## Videos related to open_manipulator_6dof_application
- [Videos for OpenManipulator SARA]()

## How to use this application?

## [1. Install ROS Packages](#install-ros-packages)
Install dependent packages for OpenManipulator SARA. Run the following command in a terminal window.

**NOTE**: The terminal application can be found with the Ubuntu search icon on the top left corner of the screen. Shortcut key for terminal is `Ctrl`+`Alt`+`t`.

``` bash
$ sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo* ros-kinetic-moveit* ros-kinetic-industrial-core
```

``` bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone https://github.com/zang09/open_manipulator_friends.git
$ git clone https://github.com/zang09/open_manipulator_6dof_simulations.git
$ git clone https://github.com/zang09/open_manipulator_6dof_application.git
$ git clone https://github.com/zang09/open_manipulator_perceptions.git
$ git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
$ cd ~/catkin_ws && catkin_make
```

If the catkin_make command has been completed without any errors, all the preparations for using OpenManipulator SARA are done.
