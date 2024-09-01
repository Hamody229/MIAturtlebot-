#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np

class KalmanFilter1D:
    def __init__(self, process_variance, measurement_variance, initial_estimate, initial_error_estimate):
        self.process_variance = process_variance          # Process variance (Q)
        self.measurement_variance = measurement_variance  # Measurement variance (R)
        self.estimate = initial_estimate                  # Initial state estimate
        self.error_estimate = initial_error_estimate      # Initial estimate of error

    def predict(self):
        # In a 1D Kalman filter, the state prediction remains the same (no control input)
        self.error_estimate += self.process_variance  # Update the error estimate with process variance

    def update(self, measurement):
        # Compute Kalman Gain
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)

        # Update the estimate with the latest measurement
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)

        # Update the error estimate
        self.error_estimate = (1 - kalman_gain) * self.error_estimate

    def get_estimate(self):
        return self.estimate

def imu_callback(data):
    # Example: Converting Quaternion to YAW angle (simplified)
    # You might need to use quaternion to Euler conversion based on your IMU data
    yaw_measurement = data.orientation.z  # This is just an example; adjust as needed
    
    kf.predict()
    kf.update(yaw_measurement)
    
    filtered_yaw = kf.get_estimate()
    filtered_yaw_pub.publish(filtered_yaw)

if __name__ == '__main__':
    rospy.init_node('kalman_filter_1d_node')

    # Set variances for process and measurement (you can tune these)
    process_variance = 0.01  # Q
    measurement_variance = 0.1  # R

    # Initial estimates
    initial_yaw_angle = 0.0
    initial_error_estimate = 1.0

    # Create Kalman filter object
    kf = KalmanFilter1D(process_variance, measurement_variance, initial_yaw_angle, initial_error_estimate)

    # Publisher for filtered YAW angle
    filtered_yaw_pub = rospy.Publisher('/filtered_yaw', Float64, queue_size=10)

    # Subscriber for IMU data
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    rospy.spin()
