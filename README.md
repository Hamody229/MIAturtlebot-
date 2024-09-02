# MiaTask10_1
***********************
## How to install GAZEBO 
***********************
https://github.com/Abdalla-El-gohary/Gazebo-Installation/tree/main

__________________________________________________________________________________________

***********************************
## Start with turtlebot 3 simulation 

first install turtlebot 3 :
```
  sudo apt-get install ros-noetic-turtlebot3-msgs
  cd ~/catkin_ws/src/
  git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  cd ~/catkin_ws && catkin_make
```


*******************************************
## Now launch the turtle model and sim world 

```
 sudo apt-get install ros-noetic-turtlebot3-description
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_gazebo turtlebot3_world.launch
```


*************************************
## How to control the bot with keyboard 

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
- Open RViz and go to the "Global Options" panel on the left.
- Set the "Fixed Frame" to map.
- Make sure that the data you want to visualize is being published relative to the map frame.
******
And then 
## Add an IMU Display:

-  Click on the "Add" button in the bottom left of the RViz window.
-  In the dialog that appears, select "By topic."
-  Browse to /imu and select the IMU display type.
-  Click "OK."

### to run the map on rviz use :
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
**********
### Add the LiDAR Data: 

- In RViz, click the Add button in the bottom left.
- Choose By topic.
- Select /scan under LaserScan.


**********
## Read IMU and convert it to YAW 

```
rosrun my_robot_package imu_to_euler_publisher.py 
```
### to preview euler data 
```
rostopic echo /imu/eular
```
