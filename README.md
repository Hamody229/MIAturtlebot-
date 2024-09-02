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

*********
## Gasussian noise 

- we will add some noise to our IMU in the URDF file by changing some numbers 
  Why adding this noise ?
    1. Simulating Real-World Conditions

       - Imperfect Sensors: In the real world, sensors like IMUs are not perfect. They have noise due to various factors like electronic interference, manufacturing imperfections, and environmental            conditions. Simulating noise helps in creating more realistic scenarios.
       - Test Robustness: Algorithms developed in perfectly clean conditions might fail in real-world noisy environments. By adding noise, you can ensure that your system or algorithm is robust and            can handle the uncertainties present in real-world data.
         
    2. Algorithm Development and Testing

        - Sensor Fusion: When combining data from multiple sensors (e.g., IMU and LiDAR), the presence of noise in one sensor can affect the fusion process. Adding noise during development allows you           to refine your sensor fusion algorithms to handle these cases effectively.
        - Filtering Techniques: Many algorithms, such as Kalman filters or complementary filters, are designed to filter out noise. Testing these algorithms with noisy data helps in tuning them for             optimal performance.
