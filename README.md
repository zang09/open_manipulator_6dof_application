# Open_manipulator_6dof_application


## Videos related to open_manipulator_6dof_application
- [Video for OpenManipulator SARA]()

## How to launch demonstration?
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
$ git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
$ cd ~/catkin_ws && catkin_make
```

If the catkin_make command has been completed without any errors, all the preparations for using OpenManipulator SARA are done.


## [2. Install Camera Packages](#install-camera-packages)

### [Realsense D435](#realsense-d435)

#### Overview
![](/images/camera/Realsense_D435.png)

The [Intel® RealSense™ Depth Camera D435](https://realsense.intel.com/depth-camera/#D415_D435) is a USB-powered depth camera and consists of a pair of depth sensors, RGB sensor, and infrared projector. It is ideal for makers and developers to add depth perception capability to their prototype development. The D435 is designed to best fit your prototype.

#### Specifications
| Items                                | Specifications                        |
|:-------------------------------------|:--------------------------------------|
| Use Environment                      | Indoor/Outdoor                        |
| RGB Sensor Resolution and Frame Rate | 1920 x 1080 at 30 fps                 |
| RGB Sensor FOV                       | 69.4°(H) x 42.5°(V) x 77°(D) (+/- 3°) |
| Depth Stream Output Resolution       | Up to 1280 x 720                      |
| Depth Stream Output Frame Rate       | Up to 90 fps                          |
| Depth Field of View (FOV)            | 85.2°(H) x 58°(V) x 94°(D) (+/- 3°)   |
| Minimum Depth Distance (Min-Z)       | 0.2m                                  |
| Maximum Range                        | Approx.10 meters                      |
| Dimension                            | 90 mm x 25 mm x 25 mm                 |
| Connectors                           | USB 3.0 Type - C                      |

#### Installation
The following commands will install relevant Intel® RealSense™ Depth Camera D435 library.
  ``` bash
  $ sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
  $ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
  $ sudo apt-get install librealsense2-dev librealsense2-utils ros-kinetic-rgbd-launch
  ```
  ``` bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/intel-ros/realsense.git
  $ cd ~/catkin_ws && catkin_make
  ```
#### Execution  
Run the following command.
  ``` bash
  $ roslaunch realsense2_camera rs_camera.launch
  ```

You can use RViz or image_view to verify driver. You can select data topic name related to Intel® RealSense™ Depth Camera D435 from drop down menu at the top of the application.
  ``` bash
  $ rqt_image_view
  ```

#### Reference
- [Intel® RealSense™ Depth Camera D435](https://realsense.intel.com/depth-camera/#D415_D435)
- [Realsense ROS package](https://github.com/intel-ros/realsense)


## [3. Install AR Marker Packages](#install-ar-marker-packages)
**NOTE**:
- This instructions were tested on `Ubuntu 16.04` and `ROS Kinetic Kame`.
- The `open_manipulator_perceptions` package requires [`ar_track_alvar`](http://wiki.ros.org/ar_track_alvar) package.
- Make sure to run the [OpenManipulator controller](/docs/en/platform/openmanipulator_x/ros_controller_package/#launch-controller) instructions before running the instructions below.

### Downlaods AR Marker
If you use the `ar_track_alvar` package to recognize the ar marker, print out the ar marker [here](http://wiki.ros.org/ar_track_alvar).

### Installation
**NOTE**:
- To use the **Raspberry Pi Camera V2**, install it on the **Remote PC**

  ``` bash
  $ sudo apt-get install ros-kinetic-ar-track-alvar ros-kinetic-ar-track-alvar-msgs ros-kinetic-image-proc
  ```
  ``` bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/zang09/open_manipulator_perceptions.git
  $ cd ~/catkin_ws && catkin_make
  ```

### Execution
You have to change the parameters according to the type of camera. Run the following command.

### Realsense D435
**NOTE**:
- [Realsense D435 ROS package](#realsense-d435) must be installed.

  ``` bash
  $ roslaunch open_manipulator_ar_markers ar_pose.launch camera_model:=realsense_d435
  ```

## [4. Launch Controller](#launch-controller)
Please, open the terminal window, run roscore as entering following command.

``` bash
$ roscore
```

After run roscore, open the other terminal window and enter the following commands in the terminal.

**NOTE**: Choose either `Controller` or `Simulation Controller` to launch your controller.
 
### 1.1. Controller
``` bash
$ roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch
```

### 1.2. Simulation Controller
``` bash
$ roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
$ roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch
```


## [5. Launch Applications](#launch-applications)
After run controller, launch GUI program to manipulate OpenManipulator SARA.

``` bash
$ roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
```

Run the following command to use camera.

``` bash
$ roslaunch open_manipulator_ar_markers ar_pose.launch
```

Finally, launch the `motion` node to perform the demonstration in the video.

``` bash
$ roslaunch open_manipulator_motion open_manipulator_motion.launch
```

When you ready, click the `START` button in the [Motion]
![](/images/GUI/GUI_motion.png)
