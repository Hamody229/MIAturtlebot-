# MIAturtlebot-
***********************
## How to install GAZEBO 
***********************
https://github.com/Abdalla-El-gohary/Gazebo-Installation/tree/main

__________________________________________________________________________________________

***********************************
## Start with turtlebot 3 simulation 
***********************************
first install turtlebot 3 :
```
  cd ~/catkin_ws/src/
  git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  cd ~/catkin_ws && catkin_make
```
### some issues you may face :
```
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "turtlebot3_msgs"
  with any of the following names:
    turtlebot3_msgsConfig.cmake
    turtlebot3_msgs-config.cmake
  Add the installation prefix of "turtlebot3_msgs" to CMAKE_PREFIX_PATH or
  set "turtlebot3_msgs_DIR" to a directory containing one of the above files.
  If "turtlebot3_msgs" provides a separate development package or SDK, be
  sure it has been installed.
Call Stack (most recent call first):
  turtlebot3_simulations/turtlebot3_fake/CMakeLists.txt:10 (find_package)
```
### How to solve it : 
```
sudo apt-get install ros-noetic-turtlebot3-msgs
```

*******************************************
Now launch the turtle model and sim world 
*******************************************
```
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### some issues you may face :

```
Resource not found: turtlebot3_description
ROS path [0]=/opt/ros/noetic/share/ros
ROS path [1]=/home/hamody/catkin_ws/src
ROS path [2]=/opt/ros/noetic/share
The traceback for the exception was written to the log file
```

### How to solve it : 
```
sudo apt-get install ros-noetic-turtlebot3-description
```

*************************************
How to control the bot with keyboard 
*************************************

```
sudo apt-get install ros-noetic-turtlebot3-teleop
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

***********************************
## Read IMU data 
***********************************
```
rostopic echo /imu
```
```
rosrun rviz rviz 
```
Open RViz and go to the "Global Options" panel on the left.
Set the "Fixed Frame" to map.
Make sure that the data you want to visualize is being published relative to the map frame.
******
And then 
Add an IMU Display:
```
  Click on the "Add" button in the bottom left of the RViz window.
  In the dialog that appears, select "By topic."
  Browse to /imu and select the IMU display type.
  Click "OK."
```
### to run the map on rviz use :
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
**********
### Add the LiDAR Data: 
```
In RViz, click the Add button in the bottom left.
Choose By topic.
Select /scan under LaserScan.
```
*********
## To publish euler data 
*********
```
rosrun my_robot_package imu_to_euler_publisher.py 
```
### And to show this data 
```
rostopic echo /imu/euler
```

