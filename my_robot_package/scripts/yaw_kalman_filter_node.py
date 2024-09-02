#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import numpy as np
filtered_yaw = 0 
class KalmanFilter1D:
    def __init__(self, process_variance, measurement_variance, initial_estimate, initial_error_estimate):
        self.process_variance = process_variance          # Process variance (Q)
        self.measurement_variance = measurement_variance  # Measurement variance (R)
        self.estimate = initial_estimate                  # Initial state estimate
        self.error_estimate = initial_error_estimate      # Initial estimate of error

    def predict(self):
        # Predict step
        self.error_estimate += self.process_variance  # Update the error estimate with process variance

    def update(self, measurement):
        # Compute Kalman Gain
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
        
        # Update the estimate with the latest measurement
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)

        # Update the error estimate
        self.error_estimate = (1 - kalman_gain) * self.error_estimate

        # Debug output to monitor the filter's operation
        rospy.loginfo(f"Measurement: {measurement}, Estimate: {self.estimate}, Kalman Gain: {kalman_gain}")

    def get_estimate(self):
        return self.estimate

def euler_callback(data):
    global filtered_yaw 
    # Use the yaw angle (Z axis) for the Kalman filter
    yaw_measurement = data.z
    
    kf.predict()
    kf.update(yaw_measurement)
    
    filtered_yaw = kf.get_estimate()

if __name__ == '__main__':
    rospy.init_node('kalman_filter_1d_node')

    # Set variances for process and measurement (you can tune these)
    process_variance = 0.09 # Q
    measurement_variance = 0.1   # R

    # Initial estimates
    initial_yaw_angle = 0.0
    initial_error_estimate = 1.0

    # Create Kalman filter object
    kf = KalmanFilter1D(process_variance, measurement_variance, initial_yaw_angle, initial_error_estimate)
    
    # Subscriber for Euler angles from the IMU
    rospy.Subscriber('/imu/euler', Vector3, euler_callback)

    # Publisher for filtered YAW angle
    filtered_yaw_pub = rospy.Publisher('/filtered_yaw', Float64, queue_size=1)
    rate = rospy.Rate(50)

    while (not rospy.is_shutdown()) :
        filtered_yaw_pub.publish(filtered_yaw)
        rate.sleep ()

    # rospy.spin()
