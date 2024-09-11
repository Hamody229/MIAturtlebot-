#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PIDController:
    def __init__(self):
        rospy.init_node('shato_pid_controller', anonymous=True)
        
        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber for odometry (to get current position)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # PID gains
        self.kp_linear = 1.0
        self.ki_linear = 0.0  
        self.kd_linear = 0.0 
      
        self.kp_angular = 4.0
        self.ki_angular = 0.0  
        self.kd_angular = 0.0 


        self.deriavitive_linear_error = 0.0
        self.deriavitive_angular_error = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        
        # Robot's current position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Target position and orientation (change this to your desired goal)
        self.x_target = 0.0 
        self.y_target = 0.0  
        self.theta_target = 0.0  
        self.rate = rospy.Rate(10)

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        
        orientation_q = data.pose.pose.orientation
        (_, _, self.theta) = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, orientation_q):
        import tf
        return tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def move_to_goal(self):
        while not rospy.is_shutdown():
            error_x = self.x_target - self.x
            error_y = self.y_target - self.y
            
            distance = math.sqrt(error_x**2 + error_y**2)
            angle_to_target = math.atan2(error_y, error_x)

            linear_proportional = self.kp_linear * distance
            angular_proportional = self.kp_angular * angular_error

      
            self.integral_linear += distance     # integeration is the sum of values over time
            self.integral_angular += angular_error          

            linear_integral = self.ki_linear * self.integral_linear
            angular_integral = self.ki_angular * self.integral_angular

            linear_derivative = self.kd_linear * (distance - self.deriavitive_linear_error)
            angular_derivative = self.kd_angular * (angular_error - self.deriavitive_angular_error)
          
            
            angular_error = self.normalize_angle(angle_to_target - self.theta)
            linear_velocity = self.kp_linear * distance
            if distance < 0.1:  # Stop when close to the target
                linear_velocity = 0.0

            angular_velocity = self.kp_angular * angular_error
            if abs(angular_error) < 0.05: 
                angular_velocity = 0.0

            vel_cmd = Twist()
            vel_cmd.linear.x = linear_velocity
            vel_cmd.angular.z = angular_velocity
            self.vel_pub.publish(vel_cmd)

            self.deriavitive_linear_error = distance
            self.deriavitive_angular_error = angular_error  # update the values to get the deriavitive          
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PIDController()
        controller.move_to_goal()
    except rospy.ROSInterruptException:
        pass
