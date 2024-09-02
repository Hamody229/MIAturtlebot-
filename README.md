# MiaTask10_1
***********************
## How to install GAZEBO 

https://github.com/Abdalla-El-gohary/Gazebo-Installation/tree/main


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
    3. How to add Gaussian noise
 
```

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git

```
-Open the URDF File: Use a text editor to open the .urdf file you downloaded. Look for the <sensor> tags that define your IMU sensors.
-change this values due to the noise you want to add
```
 <type>gaussian</type>
      <rate>
        <mean>0.0</mean>
        <stddev>1e-4</stddev>
        <bias_mean>0.0000075</bias_mean>
        <bias_stddev>1e-6</bias_stddev>
      </rate>
      <accel>
        <mean>0.0</mean>
        <stddev>1e-2</stddev>
        <bias_mean>0.005</bias_mean>
        <bias_stddev>1e-4</bias_stddev>
      </accel>
    </noise>
```
***********
## Kalman Filter 

-The Kalman Filter is a powerful mathematical tool used to estimate the state of a system by combining noisy measurements with previous estimates. It works in two phases: Prediction and Update, which rely on several key parameters.

1. State Estimate (x^kx^k​)

This is the current best estimate of the system’s state at time step kk. In your case, it is the estimate of the Yaw angle.

  - x^k−1x^k−1​: The estimate of the Yaw angle at the previous time step.
  - x^kx^k​: The updated (corrected) estimate of the Yaw angle after considering the new measurement.

2. State Covariance (PkPk​)

The state covariance PkPk​ represents the uncertainty or variance in the estimated state. A higher PkPk​ means less confidence in the estimate.

  - Pk−1Pk−1​: The uncertainty in the state estimate at the previous time step.
  - Pk−Pk−​: The predicted uncertainty before the new measurement is incorporated.
  - PkPk​: The updated uncertainty after the new measurement is incorporated.

This parameter helps the Kalman Filter "balance" how much to trust the prediction versus the new measurement.

3. Process Noise Covariance (QQ)

The process noise covariance QQ represents the uncertainty in the system model itself, i.e., how much the system (Yaw angle) can vary from the prediction due to unknown influences (e.g., sensor     drift, mechanical noise).

  - Larger QQ: Indicates more uncertainty in the process model. The Kalman Filter will trust the measurements more.
  - Smaller QQ: Indicates more confidence in the process model, and the Kalman Filter will rely more on its predictions.

*************
- ### The Two Main Steps of the Kalman Filter

- Prediction Step: In this step, the Kalman Filter predicts the next state (x^k−x^k−​) based on the previous estimate (x^k−1x^k−1​) and updates the uncertainty (covariance Pk−Pk−​).
  - x^k−=x^k−1
  - x^k−​=x^k−1​
  - Pk−=Pk−1+Q
  - Pk−​=Pk−1​+Q

- Update Step: In this step, the Kalman Filter corrects its prediction by incorporating the new measurement (zkzk​) and updates the state (x^kx^k​) and the uncertainty (PkPk​).
  - Kk=Pk−Pk−+R
  - Kk​=Pk−​+RPk−​​
  - x^k=x^k−+Kk⋅(zk−x^k−)
  - x^k​=x^k−​+Kk​⋅(zk​−x^k−​)
  - Pk=(1−Kk)⋅Pk−
  - Pk​=(1−Kk​)⋅Pk−​

- Tuning the Kalman Filter Parameters

  - Process Noise Covariance (QQ):
        If the system's actual behavior varies a lot due to external factors, you should increase QQ.
        If the system is relatively stable and predictable, use a smaller QQ.

  - Measurement Noise Covariance (RR):
        If the sensor data is very noisy, increase RR to trust the predictions more.
        If the sensor data is reliable, decrease RR so that the Kalman Filter gives more weight to the measurements.

************
## **Running Nodes and Visualizing Data with RQT**

1. **Run the Kalman Filter Node:**

   In a new terminal,  run the Kalman filter node to filter the IMU yaw data:

```
   rosrun my_robot_package yaw_kalman_filter_node.py
```

2. **Run the IMU to Euler Publisher Node:**

   In another terminal, start the IMU to Euler publisher node to convert IMU data to Euler angles and publish it:

```
   rosrun my_robot_package imu_to_euler_node.py
```   

3. **Open RQT Multiplot:**

   To visualize the data from the nodes, open RQT Multiplot in a new terminal:

```
   rqt_multiplot
```

4. **Configure the Multiplot Settings:**

   - Open the curve plot settings, then add a new curve.
   - Choose the topic for each axis: x-axis -> clock, y-axis -> imu/euler or filtered_yaw.
   - Adjust the plotting settings, such as line colors and styles, to distinguish between different data streams.
   - Set the names of the plot and the lables of the axes.

5. **Visualize the Data:**

   - Once the topics are added and configured, you should see live plots of the IMU data and filtered data.
   - Monitor the plots in real-time to analyze the data being published by your nodes.
